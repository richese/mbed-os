/**
 *******************************************************************************
 * @file    rtc.c
 * @brief Implementation of a Rtc driver
 * @internal
 * @author  ON Semiconductor
 * $Rev: 3525 $
 * $Date: 2015-07-20 15:24:25 +0530 (Mon, 20 Jul 2015) $
 ******************************************************************************
 * Copyright 2016 Semiconductor Components Industries LLC (d/b/a �ON Semiconductor�).
 * All rights reserved.  This software and/or documentation is licensed by ON Semiconductor
 * under limited terms and conditions.  The terms and conditions pertaining to the software
 * and/or documentation are available at http://www.onsemi.com/site/pdf/ONSEMI_T&C.pdf
 * (�ON Semiconductor Standard Terms and Conditions of Sale, Section 8 Software�) and
 * if applicable the software license agreement.  Do not use this software and/or
 * documentation unless you have carefully read and you agree to the limited terms and
 * conditions.  By using this software and/or documentation, you agree to the limited
 * terms and conditions.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 * @endinternal
 *
 * @ingroup rtc
 *
 * @details
 * A real-time clock (RTC) is a computer clock ,that keeps track of the current time.  The heart of the RTC is a series of
 * freely running counters one for each time unit, The series of counters is linked as follows: a roll over event of
 * the seconds counter produces a minutes enable pulse; a roll over event of the minutes counter produces an hours
 * enable pulse, etc.Note that all Counter registers are in an undefined state on power-up.
 * Use the Reset bit in the Control Register to reset the counters to their default values.
 * DIVISOR is the register containing the value to divide the clock frequency to produce 1Hz strobe ; 1Hz strobe is used
 * internally to time the incrementing of the Seconds Counter.
 * There is a set of register to set the values in the counter for each time unit.from where time is start to increment.
 * There is another set of register to set the ALARM ...Each of the Alarm Registers can be programmed with a value that
 * is used to compare to a Counter Register in order to produce an alarm (an interrupt) when the values match.
 * There is a programmable bit in each Alarm Register that determines if the alarm occurs upon a value match, or
 * if the alarm occurs upon a Counter increment condition.
 *
 */
#include "rtc.h"
#include "mbed_assert.h"
#include "lp_ticker_api.h"


//static volatile time_t last_time_read;
//
///**
// * Convert sub seconds ticks to micro seconds.
// * The clock running at 32kHz, a tick is 1/32768 of a second.
// */
//static inline uint32_t ticks_to_us(uint16_t ticks) {
//	return (((uint64_t)ticks * RTC_SEC_TO_US) / RTC_CLOCK_HZ);
//}
//
///**
// * Convert us into sub seconds ticks.
// * @note result might be troncated to be in the range [0 - RTC_SUB_SEC_MASK].
// */
//static inline uint16_t us_to_ticks(uint32_t us) {
//	return (((uint64_t) us * RTC_CLOCK_HZ) / RTC_SEC_TO_US) & RTC_SUB_SEC_MASK;
//}
//
//#define RTC_TICK_THRESHOLD 5
//


void fRtcInit(void)
{
	// Enable RTC peripheral clock to communicate with RTC.
	if (!CLOCK_IS_ENABLED(CLOCK_RTC))
	{
		CLOCK_ENABLE(CLOCK_RTC);
	}

	// Check if RTC is already counting
	if (!CLOCKREG->CCR.BITS.RTCEN)
	{
		return;
	}

	// Enable 32KHz RTC Clock.
	CLOCKREG->CCR.BITS.RTCEN = 1;

	// Wait for all RTC registers to be written.
	while (RTCREG->STATUS.BITS.BSY_ANY_WRT);

	// Reset all RTC registers.
	RTCREG->CONTROL.WORD = 0;
	RTCREG->SECOND_COUNTER = 0;
	RTCREG->SUB_SECOND_COUNTER = 0;
	RTCREG->SECOND_ALARM = 0;
	RTCREG->SUB_SECOND_ALARM = 0;
	RTCREG->STATUS.WORD = 0;
	RTCREG->INT_CLEAR.WORD = 0x3; // TODO: Create macros for 0x2
	// Wait for all RTC registers to be written.
	while (RTCREG->STATUS.BITS.BSY_ANY_WRT);

	// Enable RTC for RTC and lp_ticker functionality.
	RTCREG->CONTROL.WORD = 0x03;
}

void fRtcFree(void)
{
	CLOCK_DISABLE(CLOCK_RTC);
}

///* See rtc.h for details */
//void fRtcSetInterrupt(uint32_t timestamp)
//{
//	uint64_t current_time = fRtcRead();
//
//	uint64_t full_timestamp = (current_time & ~UINT32_MAX) | timestamp;
//	if ( (uint32_t)current_time > timestamp) {
//		full_timestamp += ((uint64_t) UINT32_MAX) + 1;
//	}
//
//	uint32_t target_seconds = full_timestamp / RTC_SEC_TO_US;
//	uint16_t target_ticks = us_to_ticks(full_timestamp);
//
//	/*
//	 * If the interrupt is in more than one second from now then use the
//	 * second alarm, otherwise use the subsecond alarm.
//	 * In case of the second alarm is used, there is no need to preserve the
//	 * remaining subsecond because the irq handler should manage spurious
//	 * interrupts (like when the timestamp is in the past). In such case, irq
//	 * handler will schedule a new interrupt with the remaining us.
//	 */
//	NVIC_DisableIRQ(Rtc_IRQn);
//	if (target_seconds != RTCREG->SECOND_COUNTER) {
//		RTCREG->SECOND_ALARM = target_seconds;
//
//		uint32_t rtc_control = RTCREG->CONTROL.WORD;
//		rtc_control |= (1 << RTC_CONTROL_SEC_CNT_INT_BIT_POS); // enable seconds interrupt
//		rtc_control &= ~(1 << RTC_CONTROL_SUBSEC_CNT_INT_BIT_POS); // disable sub sec interrupt
//		RTCREG->CONTROL.WORD = rtc_control;
//	} else {
//		uint16_t current_ticks = RTCREG->SUB_SECOND_COUNTER;
//		if (current_ticks == target_ticks ||
//			((target_ticks > current_ticks) && ((target_ticks - current_ticks) < RTC_TICK_THRESHOLD)) ||
//			((target_ticks < current_ticks) && ((RTC_SUB_SEC_MASK - (current_ticks - target_ticks)) < RTC_TICK_THRESHOLD))) {
//			// target ticks too close; schedule the interrupt immediately
//			NVIC_SetPendingIRQ(Rtc_IRQn);
//		} else {
//			RTCREG->SUB_SECOND_ALARM = target_ticks;
//
//			uint32_t rtc_control = RTCREG->CONTROL.WORD;
//			rtc_control &= ~(1 << RTC_CONTROL_SEC_CNT_INT_BIT_POS); // disable seconds interrupt
//			rtc_control |= (1 << RTC_CONTROL_SUBSEC_CNT_INT_BIT_POS); // enable sub sec interrupt
//			RTCREG->CONTROL.WORD = rtc_control;
//		}
//	}
//	 NVIC_EnableIRQ(Rtc_IRQn);
//
//     /* Wait for RTC to finish writing register - RTC operates on 32K clock as compared to 32M core*/
//     while(RTCREG->STATUS.WORD &
//        (
//        	(True << RTC_STATUS_SUB_SEC_ALARM_WRT_BIT_POS) |
//			(True << RTC_STATUS_SEC_ALARM_WRT_BIT_POS) |
//			(True << RTC_STATUS_CONTROL_WRT_BIT_POS)
//		)
//	);
//}
//
///* See rtc.h for details */
//void fRtcDisableInterrupt(void)
//{
//    NVIC_DisableIRQ(Rtc_IRQn);
//}
//
///* See rtc.h for details */
//void fRtcEnableInterrupt(void)
//{
//    NVIC_EnableIRQ(Rtc_IRQn);
//}
//
///* See rtc.h for details */
//void fRtcClearInterrupt(void)
//{
//    /* Disable subsec/sec interrupt */
//    /* Clear sec & sub_sec interrupts */
//    RTCREG->INT_CLEAR.WORD = ((True << RTC_INT_CLR_SUB_SEC_BIT_POS) |
//                              (True << RTC_INT_CLR_SEC_BIT_POS));
//
//    while((RTCREG->STATUS.WORD & ((True << RTC_STATUS_SUB_SEC_INT_CLR_WRT_BIT_POS) |
//                                  (True << RTC_STATUS_SEC_INT_CLR_WRT_BIT_POS))));  /* Wait for RTC to finish writing register - RTC operates on 32K clock as compared to 32M core*/
//}
//

time_t fRtcRead(void)
{
    /* Hardware Bug fix: The rollover of the sub-second counter initiates the increment of the second counter.
     * That means there is one cycle where the sub-second has rolled back to zero and the second counter has not incremented
     * and a read during that cycle will be incorrect.  That will occur for one RTC cycle and that is about 31us of exposure.
     * If you read a zero in the sub-second counter then increment the second counter by 1.
     * Alternatively, subtract 1 from the Sub-seconds counter to align the Second and Sub-Second rollover.
     */
	while (RTCREG->STATUS.BITS.BSY_ANY_WRT == 1);

	uint32_t seconds = RTCREG->SECOND_COUNTER;
	while (seconds != RTCREG->SECOND_COUNTER)
	{
		seconds = RTCREG->SECOND_COUNTER;
	}

	return seconds;
}

void fRtcWrite(time_t RtcTime)
{
	while (RTCREG->STATUS.BITS.BSY_ANY_WRT == 1);

    RTCREG->CONTROL.WORD = 0;
    RTCREG->SECOND_COUNTER = RtcTime;
    RTCREG->SUB_SECOND_COUNTER = 0;
    while (RTCREG->STATUS.BITS.BSY_ANY_WRT == 1);

    RTCREG->CONTROL.WORD = 0x03;
}

///* See rtc.h for details */
//void fRtcHandler(void)
//{
//    /* Disable RTC interrupt */
//    NVIC_DisableIRQ(Rtc_IRQn);
//
//    /* Clear sec & sub_sec interrupts */
//    RTCREG->INT_CLEAR.WORD = (
//    	(True << RTC_INT_CLR_SUB_SEC_BIT_POS) |
//        (True << RTC_INT_CLR_SEC_BIT_POS)
//	);
//
//    /* Disable sub seconds and seconds interrupts */
//    RTCREG->CONTROL.WORD &= ~(
//    	(True << RTC_CONTROL_SUBSEC_CNT_INT_BIT_POS) |
//        (True << RTC_CONTROL_SEC_CNT_INT_BIT_POS)
//	);
//
//	NVIC_EnableIRQ(Rtc_IRQn);
//
//    /* Wait for RTC to finish writing registers */
//    while(RTCREG->STATUS.WORD &
//    	(
//    		(True << RTC_STATUS_CONTROL_WRT_BIT_POS) |
//			(True << RTC_STATUS_SUB_SEC_INT_CLR_WRT_BIT_POS) |
//            (True << RTC_STATUS_SEC_INT_CLR_WRT_BIT_POS)
//		)
//	);
//
//	lp_ticker_irq_handler();
//}

int fIsRtcEnabled(void)
{
	if (CLOCKREG->CCR.BITS.RTCEN)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
