#ifndef __SYSTEM_SWM166_H__
#define __SYSTEM_SWM166_H__

#ifdef __cplusplus
 extern "C" {
#endif


extern uint32_t SystemCoreClock;		// System Clock Frequency (Core Clock)
extern uint32_t CyclesPerUs;			// Cycles per micro second


extern void SystemInit(void);

extern void SystemCoreClockUpdate (void);

extern void PLLInit(void);

extern void switchTo12MHz(void);
extern void switchTo1M5Hz(void);
extern void switchToXTAL(uint32_t div8);
extern void switchToPLL(uint32_t div8);
extern void switchTo32KHz(void);


#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_SWM166_H__
