/****************************************************************************************************************************************** 
* �ļ�����:	system_SWM166.c
* ����˵��:	SWM166��Ƭ����ʱ������
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����: V1.0.0		2016��1��30��
* ������¼: 
*
*
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology
*******************************************************************************************************************************************/ 
#include <stdint.h>
#include "SWM166.h"


/******************************************************************************************************************************************
 * ϵͳʱ���趨
 *****************************************************************************************************************************************/
#define SYS_CLK_12MHz		0	 	//�ڲ���Ƶ 12MHz RC����
#define SYS_CLK_1M5Hz		1		//�ڲ���Ƶ1.5MHz RC����
#define SYS_CLK_XTAL		2		//�ⲿ����������2-32MHz��
#define SYS_CLK_XTAL_DIV8	3		//�ⲿ����������2-32MHz�� 8��Ƶ
#define SYS_CLK_PLL			4		//���໷���
#define SYS_CLK_PLL_DIV8	5		//���໷��� 8��Ƶ
#define SYS_CLK_32KHz		6		//�ڲ���Ƶ32KHz RC  ����

#define SYS_CLK   SYS_CLK_PLL


#define __HSI		(12000000UL)		//�����ڲ�ʱ��
#define __LSI		(   32000UL)		//�����ڲ�ʱ��
#define __HSE		(12000000UL)		//�����ⲿʱ��
#define __LSE		(   32000UL)		//�����ⲿʱ��


/********************************** PLL �趨 **********************************************
 * VCO���Ƶ�� = PLL����ʱ�� / INDIV * 4 * FBDIV
 * PLL���Ƶ�� = PLL����ʱ�� / INDIV * 4 * FBDIV / OUTDIV = VCO���Ƶ�� / OUTDIV         
 *****************************************************************************************/ 
#define SYS_PLL_SRC   	SYS_CLK_XTAL	//��ȡֵSYS_CLK_12MHz��SYS_CLK_XTAL

#define PLL_IN_DIV		3

#define PLL_FB_DIV		30


#define PLL_OUT_DIV8	0
#define PLL_OUT_DIV4	1
#define PLL_OUT_DIV2	2

#define PLL_OUT_DIV		PLL_OUT_DIV8


uint32_t SystemCoreClock  = __HSI;   				//System Clock Frequency (Core Clock)
uint32_t CyclesPerUs      = (__HSI / 1000000); 		//Cycles per micro second


/****************************************************************************************************************************************** 
* ��������: 
* ����˵��: This function is used to update the variable SystemCoreClock and must be called whenever the core clock is changed
* ��    ��: 
* ��    ��: 
* ע������: 
******************************************************************************************************************************************/
void SystemCoreClockUpdate(void)    
{
	if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)			//SYS  <= HRC
	{
		SystemCoreClock = __HSI;
	}
	else											//SYS  <= CLK
	{
		switch((SYS->CLKSEL & SYS_CLKSEL_CLK_Msk) >> SYS_CLKSEL_CLK_Pos)
		{
		case 0:
			SystemCoreClock = __LSI;
			break;
		
		case 1:
			if(SYS->PLLCR & SYS_PLLCR_INSEL_Msk)		//PLL_IN <= HRC
			{
				SystemCoreClock = __HSI;
			}
			else										//PLL_IN <= XTAL
			{
				SystemCoreClock = __HSE;
			}
			
			SystemCoreClock = SystemCoreClock / PLL_IN_DIV * PLL_FB_DIV * 4 / (2 << (2 - PLL_OUT_DIV));
			break;
		
		case 3:
			SystemCoreClock = __HSE;
			break;
		
		case 4:
			SystemCoreClock = __HSI;
			break;
		}
		
		if(SYS->CLKSEL & SYS_CLKSEL_CLK_DIVx_Msk)  SystemCoreClock /= 8;
	}
	
	CyclesPerUs = SystemCoreClock / 1000000;
}

/****************************************************************************************************************************************** 
* ��������: 
* ����˵��: The necessary initializaiton of systerm
* ��    ��: 
* ��    ��: 
* ע������: 
******************************************************************************************************************************************/
void SystemInit(void)
{
	SYS->CLKEN0 |= (1 << SYS_CLKEN0_ANAC_Pos);
	
	if((SYS->BACKUP[3] >> 29) == 5)
		*((__IO uint32_t *)((uint32_t)&SYS->HRCCR + 8)) = SYS->BACKUP[3] & (~(7u << 29));
	
	Flash_Param_at_xMHz(90);
	
	switch(SYS_CLK)
	{
		case SYS_CLK_12MHz:
			switchTo12MHz();
			break;
		
		case SYS_CLK_1M5Hz:
			switchTo1M5Hz();
			break;
		
		case SYS_CLK_XTAL:
			switchToXTAL(0);
			break;
		
		case SYS_CLK_XTAL_DIV8:
			switchToXTAL(1);
			break;
		
		case SYS_CLK_PLL:
			switchToPLL(0);
			break;
		
		case SYS_CLK_PLL_DIV8:
			switchToPLL(1);
			break;

		case SYS_CLK_32KHz:
			switchTo32KHz();
			break;
	}
	
	SystemCoreClockUpdate();
	
	Flash_Param_at_xMHz(SystemCoreClock/1000000);
	
	FMC->CACHE = (1 << FMC_CACHE_CEN_Pos) | (1 << FMC_CACHE_CCLR_Pos);	// ��� Cache
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	FMC->CACHE = (1 << FMC_CACHE_CEN_Pos) | (1 << FMC_CACHE_CPEN_Pos);
	
	PORTB->PULLD &= ~((1 << PIN10) | (1 << PIN11));
	PORTB->PULLU &= ~((1 << PIN12) | (1 << PIN14));
	
	SYS->PGACR &= ~SYS_PGACR_VRTRIM_Msk;	//PGA������˲ο���ѹУ׼
	SYS->PGACR |= ((SYS->CHIPID[3] & 0x000F) << SYS_PGACR_VRTRIM_Pos);

	SYS->PGACR &= ~SYS_PGACR_DATRIM_Msk;	//���ڲ���ACMP N����˵�ѹ��DAC�Ĳο���ѹУ׼
	SYS->PGACR |= (((SYS->CHIPID[3] >> 16) & 0x000F) << SYS_PGACR_DATRIM_Pos);
}

void switchTo12MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_ON_Pos) |
				 (0 << SYS_HRCCR_DBL_Pos);			//HRC = 12MHz
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC
}

void switchTo1M5Hz(void)
{
	switchTo12MHz();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (4 << SYS_CLKSEL_CLK_Pos);		//CLK <= HRC

	SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= HRC/8
}

void switchToXTAL(uint32_t div8)
{
	uint32_t i;
	
	switchTo12MHz();
	
	PORTB->PULLU &= ~((1 << PIN11) | (1 << PIN12));
	PORTB->PULLD &= ~((1 << PIN11) | (1 << PIN12));
	PORT_Init(PORTB, PIN11, PORTB_PIN11_XTAL_IN,  0);
	PORT_Init(PORTB, PIN12, PORTB_PIN12_XTAL_OUT, 0);
	SYS->XTALCR |= (1 << SYS_XTALCR_ON_Pos) | (7 << SYS_XTALCR_DRV_Pos) | (1 << SYS_XTALCR_DET_Pos);
	for(i = 0; i < 1000; i++) __NOP();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (3 << SYS_CLKSEL_CLK_Pos);		//CLK <= XTAL

	if(div8) SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	else     SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= XTAL
}

void switchToPLL(uint32_t div8)
{
	switchTo12MHz();
	
	PLLInit();
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_Pos);		//CLK <= PLL

	if(div8)  SYS->CLKSEL |= (1 << SYS_CLKSEL_CLK_DIVx_Pos);
	else      SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);
	
	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= PLL
}

void switchTo32KHz(void)
{
	switchTo12MHz();
	
	SYS->LRCCR = (1 << SYS_LRCCR_ON_Pos);
	
	SYS->CLKDIVx_ON = 1;
	
	SYS->CLKSEL &= ~SYS_CLKSEL_CLK_Msk;
	SYS->CLKSEL |= (0 << SYS_CLKSEL_CLK_Pos);		//CLK <= LRC

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_CLK_DIVx_Pos);

	SYS->CLKSEL &=~(1 << SYS_CLKSEL_SYS_Pos);		//SYS <= LRC
}


void PLLInit(void)
{
	uint32_t i;
	
	if(SYS_PLL_SRC == SYS_CLK_12MHz)
	{
		SYS->HRCCR = (1 << SYS_HRCCR_ON_Pos) |
					 (0 << SYS_HRCCR_DBL_Pos);		//HRC = 12Hz
		
		SYS->PLLCR |= (1 << SYS_PLLCR_INSEL_Pos);	//PLL_SRC <= HRC
	}
	else if(SYS_PLL_SRC == SYS_CLK_XTAL)
	{
		PORTB->PULLU &= ~((1 << PIN11) | (1 << PIN12));
		PORTB->PULLD &= ~((1 << PIN11) | (1 << PIN12));
		PORT_Init(PORTB, PIN11, PORTB_PIN11_XTAL_IN,  0);
		PORT_Init(PORTB, PIN12, PORTB_PIN12_XTAL_OUT, 0);
		SYS->XTALCR |= (1 << SYS_XTALCR_ON_Pos) | (7 << SYS_XTALCR_DRV_Pos) | (1 << SYS_XTALCR_DET_Pos);
		for(i = 0; i < 1000; i++) __NOP();
		
		SYS->PLLCR &= ~(1 << SYS_PLLCR_INSEL_Pos);	//PLL_SRC <= XTAL
	}
	
	SYS->PLLDIV &= ~(SYS_PLLDIV_INDIV_Msk |
					 SYS_PLLDIV_FBDIV_Msk |
					 SYS_PLLDIV_OUTDIV_Msk);
	SYS->PLLDIV |= (PLL_IN_DIV << SYS_PLLDIV_INDIV_Pos) |
				   (PLL_FB_DIV << SYS_PLLDIV_FBDIV_Pos) |
				   (PLL_OUT_DIV<< SYS_PLLDIV_OUTDIV_Pos);
	
	SYS->PLLCR &= ~(1 << SYS_PLLCR_OFF_Pos);
	
	while(SYS->PLLLOCK == 0);		//�ȴ�PLL����
	
	SYS->PLLCR |= (1 << SYS_PLLCR_OUTEN_Pos);
}
