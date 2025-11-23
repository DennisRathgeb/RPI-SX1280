#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xdcb764ad, "memset" },
	{ 0x54b7682a, "spi_sync" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x25df42d5, "__spi_register_driver" },
	{ 0x92893115, "driver_unregister" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x105a915b, "gpiod_get_value" },
	{ 0x45504387, "_dev_err" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x4829a47e, "memcpy" },
	{ 0x36a78de3, "devm_kmalloc" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x5a1c908c, "devm_gpiod_get_optional" },
	{ 0x3ce80115, "devm_request_threaded_irq" },
	{ 0x590ced7e, "_dev_info" },
	{ 0x474e54d2, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Csemtech,sx1280");
MODULE_ALIAS("of:N*T*Csemtech,sx1280C*");

MODULE_INFO(srcversion, "71209757EF6920ECC9BD3CC");
