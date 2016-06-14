#include <common.h>

DECLARE_GLOBAL_DATA_PTR;

#if (CONFIG_USE_A9_GLOBAL_TIMER == 1)
/*a9 cpu freq*/
#define A9_APU_FREQ 	666666687
#define A9_GT_BASE	(0xf8f00200)
#endif

int timer_init(void)
{
#if (CONFIG_USE_A9_GLOBAL_TIMER == 1)
	gd->arch.timer_rate_hz = (A9_APU_FREQ / 2) / CONFIG_SYS_HZ;
#elif (CONFIG_USE_HW_TIMER==1)
	gd->arch.timer_rate_hz = 1;
#endif

	return 0;
}

/* get timer counter*/
unsigned long long get_ticks(void)
{
	unsigned long nowl,nowu;

#if (CONFIG_USE_A9_GLOBAL_TIMER == 1)
	nowl = (*(volatile unsigned long*)(A9_GT_BASE));
	nowu = (*(volatile unsigned long*)(A9_GT_BASE + 4));
#elif (CONFIG_USE_HW_TIMER==1)

#endif

	return (((unsigned long long)nowu) << 32) | nowl;
}

ulong get_tbclk(void)
{
	return gd->arch.timer_rate_hz;
}
