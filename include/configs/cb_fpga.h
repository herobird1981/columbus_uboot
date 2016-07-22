#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_USE_A9_GLOBAL_TIMER	1
#define CONFIG_USE_HW_TIMER		0

#define CONFIG_SYS_SDRAM_SIZE		SZ_256M

#define DEFAULT_PCLK	50000000

/*
#define CONFIG_BOOTARGS	"console=ttyS0,115200 earlyprintk "
*/

/*board specifiy environment*/
#define CONFIG_EXTRA_ENV_SETTINGS_BOARD \
        "verify=no\0" \
        "usbnet_devaddr=00:00:11:22:33:44\0" \
        "usbnet_hostaddr=00:00:11:22:33:55\0" \
        "ethaddr=00:00:11:22:33:66\0"

#define CONFIG_SERVERIP  192.168.1.100 /*env: serverip*/
#define CONFIG_IPADDR  192.168.1.200 /*env: ipaddr*/

#include <configs/columbus.h>

#endif
