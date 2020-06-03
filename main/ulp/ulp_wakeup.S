#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	.global rtc_gpio_mask
rtc_gpio_mask:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	// Load GPIO Mask into R1
	move r3,rtc_gpio_mask
        ld r1,r3,0

	// Load bits for RTC_GPIO 0 - 15 into R0
        reg_rd RTC_GPIO_IN_REG,29,14

	// Apply Mask - If nothing pressed then the result will be the same as the mask
	and r0,r0,r1

	// Subtract the value of the mask
	sub r0,r0,r1

	// If the result of the subtraction is zero then nothing was pressed (exit)
	jump exit, eq

	// If the result was non-zero then something must have been pressed (wakeup)

	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)

	.global exit
exit:
	halt
