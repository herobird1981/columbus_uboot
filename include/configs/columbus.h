#ifndef __COLUMBUS_H__
#define __COLUMBUS_H__

#include <linux/sizes.h>

#define CONFIG_SYS_CACHELINE_SIZE	32

/* uboot size limit:1M */
#define CONFIG_BOARD_SIZE_LIMIT		1048576

#define CONFIG_SYS_BOOTM_LEN 0x1000000

#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_TEXT_BASE		0x83f80000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_INIT_SP_ADDR 	(CONFIG_SYS_TEXT_BASE + SZ_4M - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_MALLOC_LEN		(32 << 20)

#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			SZ_128K

#define CONFIG_DT_LOAD_ADDR		0x84080000
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_EXTRA_ENV_SETTINGS_BOARD \
	"fdtcontroladdr=" __stringify(CONFIG_DT_LOAD_ADDR) "\0" \
	"bootfile=kernel.itb\0"

#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_SYS_MAXARGS		16

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_NS16550_MEM32

/* No NOR flash */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R

#define CONFIG_OF_LIBFDT

#define CONFIG_CQSPI_REF_CLK		(plat->ref_clk)
#define CONFIG_CQSPI_DECODER		0

/*USB SUPPORT*/
#define CONFIG_USB_MUSB_PIO_ONLY

/*USB DEV SUPPORT*/
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_GADGET_DUALSPEED

#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_G_DNL_VENDOR_NUM	0x03FD
#define CONFIG_G_DNL_PRODUCT_NUM	0x0300
#define CONFIG_G_DNL_MANUFACTURER	"Itron"

#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_RNDIS

/*usb dev mas storage class support*/
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE
/* nand */
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE 0
#define CONFIG_SYS_NAND_SELF_INIT
#define BRITE_NAND_ECC_SIZE	512
#define BRITE_NAND_ECC_STRENGTH	8

#endif
