#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "sx1280_driver.h"

struct sx1280 {
    struct spi_device *spi;
    struct gpio_desc *reset_gpiod;
    struct gpio_desc *busy_gpiod;
    struct gpio_desc *txen_gpiod;
    struct gpio_desc *rxen_gpiod;
    int irq;
    struct mutex lock;
    /* radio state, buffers, etc. */
};

static int sx1280_spi_xfer(struct sx1280 *dev,
                           const void *tx, void *rx, size_t len)
{
    struct spi_transfer t = {
        .tx_buf = tx,
        .rx_buf = rx,
        .len    = len,
    };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    return spi_sync(dev->spi, &m);
}

/*
 * ============================================================================
 * Low-Level SPI Communication Functions (Stage 1)
 * ============================================================================
 */

/**
 * sx1280_wait_busy() - Wait for BUSY pin to go LOW
 * @dev: SX1280 device structure
 * @timeout_ms: Timeout in milliseconds
 *
 * The BUSY pin goes HIGH when the radio is processing a command.
 * We must wait for it to go LOW before starting a new SPI transaction.
 *
 * Return: 0 on success, -ETIMEDOUT if BUSY doesn't clear within timeout
 */
int sx1280_wait_busy(struct sx1280 *dev, unsigned int timeout_ms)
{
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(timeout_ms);

    /* Check if busy_gpiod is configured */
    if (!dev->busy_gpiod)
        return 0;  /* No BUSY pin, assume ready */

    /* Poll BUSY pin until it goes LOW or timeout */
    while (gpiod_get_value(dev->busy_gpiod)) {
        if (time_after(jiffies, timeout_jiffies)) {
            dev_err(&dev->spi->dev, "Timeout waiting for BUSY pin\n");
            return -ETIMEDOUT;
        }
        usleep_range(10, 20);  /* Sleep 10-20us between polls */
    }

    return 0;
}

/**
 * sx1280_write_command() - Send a command with optional parameters
 * @dev: SX1280 device structure
 * @command: Command opcode
 * @params: Parameter data to send (can be NULL if param_len is 0)
 * @param_len: Number of parameter bytes
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW (automatic with SPI transfer)
 *   3. Send command opcode
 *   4. Send parameter bytes
 *   5. NSS goes HIGH (automatic)
 *   6. Wait for BUSY to be LOW (except for SLEEP command)
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_write_command(struct sx1280 *dev, u8 command,
                         const u8 *params, size_t param_len)
{
    u8 tx_buf[256];  /* Command + up to 255 parameter bytes */
    int ret;

    /* Validate parameter length */
    if (param_len > 255)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Build transmit buffer: [command][param0][param1]...[paramN] */
    tx_buf[0] = command;
    if (params && param_len > 0)
        memcpy(&tx_buf[1], params, param_len);

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, NULL, 1 + param_len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /*
     * After command, wait for BUSY to clear again
     * Exception: SLEEP command doesn't pull BUSY high
     */
    if (command != RADIO_SET_SLEEP) {
        ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
        if (ret)
            return ret;
    }

    return 0;
}

/**
 * sx1280_read_command() - Send a command and read response data
 * @dev: SX1280 device structure
 * @command: Command opcode
 * @data: Buffer to store received data
 * @data_len: Number of bytes to read
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW
 *   3. Send command opcode
 *   4. Send NOP byte (0x00) - required by SX1280 protocol
 *   5. Read data bytes (send 0x00 and capture MISO)
 *   6. NSS goes HIGH
 *   7. Wait for BUSY to be LOW
 *
 * Special case: RADIO_GET_STATUS doesn't need the NOP byte,
 * and returns status in the first byte itself.
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_read_command(struct sx1280 *dev, u8 command,
                        u8 *data, size_t data_len)
{
    u8 tx_buf[258];  /* Command + NOP + up to 255 data bytes */
    u8 rx_buf[258];
    size_t total_len;
    int ret;

    /* Validate data length */
    if (data_len > 255)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Prepare transmit buffer */
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));
    tx_buf[0] = command;

    /*
     * RADIO_GET_STATUS is special:
     * Send: [0xC0] [0x00] [0x00]
     * Recv: [status] [don't care] [don't care]
     */
    if (command == RADIO_GET_STATUS) {
        total_len = 3;  /* Command + 2 dummy bytes */
    } else {
        /* Normal read: [command] [NOP] [data0] [data1] ... */
        total_len = 1 + 1 + data_len;  /* Command + NOP + data */
    }

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, rx_buf, total_len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /* Extract received data */
    if (command == RADIO_GET_STATUS) {
        /* Status is in the first byte */
        data[0] = rx_buf[0];
    } else {
        /* Data starts after command + NOP bytes */
        memcpy(data, &rx_buf[2], data_len);
    }

    /* Wait for BUSY to clear */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    return 0;
}

/**
 * sx1280_write_register() - Write to one or more registers
 * @dev: SX1280 device structure
 * @address: 16-bit register address
 * @data: Data to write
 * @len: Number of bytes to write
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW
 *   3. Send RADIO_WRITE_REGISTER (0x18)
 *   4. Send address high byte
 *   5. Send address low byte
 *   6. Send data bytes
 *   7. NSS goes HIGH
 *   8. Wait for BUSY to be LOW
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_write_register(struct sx1280 *dev, u16 address,
                          const u8 *data, size_t len)
{
    u8 tx_buf[258];  /* Command + addr_hi + addr_lo + up to 255 data bytes */
    int ret;

    /* Validate data length */
    if (len > 255)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Build transmit buffer: [WRITE_REG][addr_hi][addr_lo][data0]...[dataN] */
    tx_buf[0] = RADIO_WRITE_REGISTER;
    tx_buf[1] = (address >> 8) & 0xFF;  /* Address high byte */
    tx_buf[2] = address & 0xFF;          /* Address low byte */
    memcpy(&tx_buf[3], data, len);

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, NULL, 3 + len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /* Wait for BUSY to clear */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    return 0;
}

/**
 * sx1280_read_register() - Read from one or more registers
 * @dev: SX1280 device structure
 * @address: 16-bit register address
 * @data: Buffer to store read data
 * @len: Number of bytes to read
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW
 *   3. Send RADIO_READ_REGISTER (0x19)
 *   4. Send address high byte
 *   5. Send address low byte
 *   6. Send dummy byte (0x00)
 *   7. Read data bytes
 *   8. NSS goes HIGH
 *   9. Wait for BUSY to be LOW
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_read_register(struct sx1280 *dev, u16 address,
                         u8 *data, size_t len)
{
    u8 tx_buf[259];  /* Command + addr_hi + addr_lo + dummy + up to 255 data bytes */
    u8 rx_buf[259];
    int ret;

    /* Validate data length */
    if (len > 255)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Build transmit buffer: [READ_REG][addr_hi][addr_lo][dummy][0x00]...[0x00] */
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));
    tx_buf[0] = RADIO_READ_REGISTER;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;
    /* tx_buf[3] is already 0x00 (dummy byte) */

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, rx_buf, 4 + len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /* Extract data (skip command + addr_hi + addr_lo + dummy bytes) */
    memcpy(data, &rx_buf[4], len);

    /* Wait for BUSY to clear */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    return 0;
}

/**
 * sx1280_write_buffer() - Write data to TX buffer
 * @dev: SX1280 device structure
 * @offset: Buffer offset to start writing
 * @data: Payload data to write
 * @len: Number of bytes to write
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW
 *   3. Send RADIO_WRITE_BUFFER (0x1A)
 *   4. Send offset byte
 *   5. Send payload data
 *   6. NSS goes HIGH
 *   7. Wait for BUSY to be LOW
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_write_buffer(struct sx1280 *dev, u8 offset,
                        const u8 *data, size_t len)
{
    u8 tx_buf[257];  /* Command + offset + up to 255 data bytes */
    int ret;

    /* Validate data length */
    if (len > SX1280_MAX_PAYLOAD_LENGTH)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Build transmit buffer: [WRITE_BUFFER][offset][data0]...[dataN] */
    tx_buf[0] = RADIO_WRITE_BUFFER;
    tx_buf[1] = offset;
    memcpy(&tx_buf[2], data, len);

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, NULL, 2 + len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /* Wait for BUSY to clear */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    return 0;
}

/**
 * sx1280_read_buffer() - Read data from RX buffer
 * @dev: SX1280 device structure
 * @offset: Buffer offset to start reading
 * @data: Buffer to store received payload
 * @len: Number of bytes to read
 *
 * SPI Transaction:
 *   1. Wait for BUSY to be LOW
 *   2. NSS goes LOW
 *   3. Send RADIO_READ_BUFFER (0x1B)
 *   4. Send offset byte
 *   5. Send NOP byte (0x00)
 *   6. Read payload data
 *   7. NSS goes HIGH
 *   8. Wait for BUSY to be LOW
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_read_buffer(struct sx1280 *dev, u8 offset,
                       u8 *data, size_t len)
{
    u8 tx_buf[258];  /* Command + offset + NOP + up to 255 data bytes */
    u8 rx_buf[258];
    int ret;

    /* Validate data length */
    if (len > SX1280_MAX_PAYLOAD_LENGTH)
        return -EINVAL;

    /* Wait for radio to be ready */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    /* Build transmit buffer: [READ_BUFFER][offset][NOP][0x00]...[0x00] */
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));
    tx_buf[0] = RADIO_READ_BUFFER;
    tx_buf[1] = offset;
    /* tx_buf[2] is already 0x00 (NOP byte) */

    /* Perform SPI transfer */
    mutex_lock(&dev->lock);
    ret = sx1280_spi_xfer(dev, tx_buf, rx_buf, 3 + len);
    mutex_unlock(&dev->lock);

    if (ret) {
        dev_err(&dev->spi->dev, "SPI transfer failed: %d\n", ret);
        return ret;
    }

    /* Extract data (skip command + offset + NOP bytes) */
    memcpy(data, &rx_buf[3], len);

    /* Wait for BUSY to clear */
    ret = sx1280_wait_busy(dev, SX1280_BUSY_TIMEOUT_MS);
    if (ret)
        return ret;

    return 0;
}

/**
 * sx1280_get_status() - Read radio status
 * @dev: SX1280 device structure
 * @status: Pointer to store status byte
 *
 * This is a simple test function to verify SPI communication works.
 * The status byte contains chip mode and command status information.
 *
 * Return: 0 on success, negative error code on failure
 */
int sx1280_get_status(struct sx1280 *dev, u8 *status)
{
    return sx1280_read_command(dev, RADIO_GET_STATUS, status, 1);
}

static irqreturn_t sx1280_irq_thread(int irq, void *data)
{
    struct sx1280 *dev = data;

    /* read IRQ status from chip, clear flags, push packet up, etc. */
    return IRQ_HANDLED;
}

static int sx1280_probe(struct spi_device *spi)
{
    struct sx1280 *dev;
    int ret;

    dev = devm_kzalloc(&spi->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->spi = spi;
    mutex_init(&dev->lock);

    dev->reset_gpiod = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
    dev->busy_gpiod  = devm_gpiod_get_optional(&spi->dev, "busy", GPIOD_IN);
    dev->txen_gpiod  = devm_gpiod_get_optional(&spi->dev, "txen", GPIOD_OUT_LOW);
    dev->rxen_gpiod  = devm_gpiod_get_optional(&spi->dev, "rxen", GPIOD_OUT_LOW);

    /* request IRQ from DT */
    dev->irq = spi->irq;
    ret = devm_request_threaded_irq(&spi->dev, dev->irq,
                                    NULL, sx1280_irq_thread,
                                    IRQF_ONESHOT, "sx1280", dev);
    if (ret)
        return ret;

    spi_set_drvdata(spi, dev);

    /*
     * Test basic SPI communication by reading status register
     * This verifies that:
     * - SPI bus is working
     * - BUSY pin is functional
     * - SX1280 is responding
     */
    u8 status = 0;
    ret = sx1280_get_status(dev, &status);
    if (ret) {
        dev_err(&spi->dev, "Failed to read status register: %d\n", ret);
        dev_err(&spi->dev, "Check SPI connections and BUSY pin\n");
        return ret;
    }

    /* Decode status byte */
    u8 chip_mode = (status & STATUS_MODE_MASK) >> STATUS_MODE_SHIFT;
    u8 cmd_status = (status & STATUS_CMD_MASK) >> STATUS_CMD_SHIFT;

    dev_info(&spi->dev, "SX1280 detected! Status=0x%02x\n", status);
    dev_info(&spi->dev, "  Chip Mode: 0x%x ", chip_mode);

    /* Print human-readable mode */
    switch (status & STATUS_MODE_MASK) {
    case STATUS_MODE_STDBY_RC:
        dev_info(&spi->dev, "(STANDBY_RC)\n");
        break;
    case STATUS_MODE_STDBY_XOSC:
        dev_info(&spi->dev, "(STANDBY_XOSC)\n");
        break;
    case STATUS_MODE_FS:
        dev_info(&spi->dev, "(FS)\n");
        break;
    case STATUS_MODE_RX:
        dev_info(&spi->dev, "(RX)\n");
        break;
    case STATUS_MODE_TX:
        dev_info(&spi->dev, "(TX)\n");
        break;
    default:
        dev_info(&spi->dev, "(UNKNOWN)\n");
        break;
    }

    dev_info(&spi->dev, "  Command Status: 0x%x\n", cmd_status);
    dev_info(&spi->dev, "SPI communication test PASSED\n");

    /* TODO: Perform chip reset and initialization (Stage 2) */

    return 0;
}

static void sx1280_remove(struct spi_device *spi)
{
    struct sx1280 *dev = spi_get_drvdata(spi);
    /* power down radio etc. (devm_* cleanup is automatic) */
}

static const struct of_device_id sx1280_of_match[] = {
    { .compatible = "semtech,sx1280" },
    { }
};
MODULE_DEVICE_TABLE(of, sx1280_of_match);

static struct spi_driver sx1280_driver = {
    .driver = {
        .name           = "sx1280",
        .of_match_table = sx1280_of_match,
    },
    .probe  = sx1280_probe,
    .remove = sx1280_remove,
};

module_spi_driver(sx1280_driver);

MODULE_AUTHOR("Dennis’ future self");
MODULE_DESCRIPTION("Semtech SX1280 SPI radio driver (minimal)");
MODULE_LICENSE("GPL");
