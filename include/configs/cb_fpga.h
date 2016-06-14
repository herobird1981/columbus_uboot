#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_USE_A9_GLOBAL_TIMER	1
#define CONFIG_USE_HW_TIMER		0

#define CONFIG_SYS_SDRAM_SIZE		SZ_256M

/*
#define CONFIG_BOOTARGS	"console=ttyS0,115200 earlyprintk "
*/

/*board specifiy environment*/
#define CONFIG_EXTRA_ENV_SETTINGS_BOARD \
        "verify=no\0"

#include <configs/columbus.h>

#endif
