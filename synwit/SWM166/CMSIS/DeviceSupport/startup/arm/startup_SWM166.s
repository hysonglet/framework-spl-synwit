;******************************************************************************************************************************************
; �ļ�����:	startup_SWM166.s
; ����˵��:	SWM166��Ƭ���������ļ�
; ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
; ע������:
; �汾����: V1.0.0		2016��1��30��
; ������¼:
;
;
;******************************************************************************************************************************************
; @attention
;
; THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION
; REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE
; FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
; OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
; -ECTION WITH THEIR PRODUCTS.
;
; COPYRIGHT 2012 Synwit Technology
;******************************************************************************************************************************************


; Amount of memory (in bytes) allocated for Stack
Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; Amount of memory (in bytes) allocated for Heap
Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, CODE, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD    Reset_Handler              ; Reset Handler
                DCD    NMI_Handler                ; NMI Handler
                DCD    HardFault_Handler          ; Hard Fault Handler
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0
                DCD    SVC_Handler                ; SVCall Handler
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    PendSV_Handler             ; PendSV Handler
                DCD    SysTick_Handler            ; SysTick Handler

                ; External Interrupts
				DCD    UART0_Handler
				DCD    TIMR0_Handler
				DCD    CORDIC_Handler
				DCD    UART1_Handler
				DCD    PWM1_Handler
				DCD    TIMR1_Handler
				DCD    HALL_Handler
				DCD    PWM0_Handler
				DCD    BOD_Handler
				DCD    PWMBRK_Handler
				DCD    0
				DCD    WDT_Handler
				DCD    I2C0_Handler
				DCD    XTALSTOP_Handler
				DCD    ADC0_Handler
				DCD    ACMP_Handler
				DCD    BTIMR0_Handler
				DCD    BTIMR1_Handler
				DCD    BTIMR2_Handler
				DCD    BTIMR3_Handler
				DCD    GPIOA_Handler
				DCD    GPIOB_Handler
				DCD    GPIOM_Handler
				DCD    GPIOA0_GPIOM0_SPI1_Handler
				DCD    GPIOA1_GPIOM1_MPU_Handler
				DCD    GPIOA2_GPIOM2_Handler
				DCD    GPIOA3_GPIOM3_Handler
				DCD    GPIOB0_GPIOA8_TIMR2_Handler
				DCD    GPIOB1_GPIOA9_DMA_Handler
				DCD    GPIOB2_GPIOA10_CAN0_Handler
				DCD    GPIOB3_GPIOA11_SPI0_Handler
				DCD    GPIOB4_GPIOB10_QEI_Handler   

__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors



                ;AREA    |.text|, CODE, READONLY

Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
				 LDR R0,=0x4004A00C
				 LDR R1,=0x00050000
				 STR R1, [R0]
				 IMPORT  __main
                 LDR     R0, =__main
                 BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP

HardFault_Handler PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
				
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP

PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP

SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

UART0_Handler  PROC
                EXPORT  UART0_Handler              [WEAK]
                B       .
                ENDP

TIMR0_Handler  PROC
                EXPORT  TIMR0_Handler              [WEAK]
                B       .
                ENDP

CORDIC_Handler  PROC
                EXPORT  CORDIC_Handler             [WEAK]
                B       .
                ENDP

UART1_Handler  PROC
                EXPORT  UART1_Handler              [WEAK]
                B       .
                ENDP

PWM1_Handler  PROC
                EXPORT  PWM1_Handler               [WEAK]
                B       .
                ENDP

TIMR1_Handler  PROC
                EXPORT  TIMR1_Handler              [WEAK]
                B       .
                ENDP

HALL_Handler  PROC
                EXPORT  HALL_Handler               [WEAK]
                B       .
                ENDP

PWM0_Handler  PROC
                EXPORT  PWM0_Handler               [WEAK]
                B       .
                ENDP

BOD_Handler  PROC
                EXPORT  BOD_Handler                [WEAK]
                B       .
                ENDP

PWMBRK_Handler  PROC
                EXPORT  PWMBRK_Handler             [WEAK]
                B       .
                ENDP

WDT_Handler  PROC
                EXPORT  WDT_Handler                [WEAK]
                B       .
                ENDP

I2C0_Handler  PROC
                EXPORT  I2C0_Handler               [WEAK]
                B       .
                ENDP

XTALSTOP_Handler  PROC
                EXPORT  XTALSTOP_Handler           [WEAK]
                B       .
                ENDP

ADC0_Handler  PROC
                EXPORT  ADC0_Handler               [WEAK]
                B       .
                ENDP

ACMP_Handler  PROC
                EXPORT  ACMP_Handler               [WEAK]
                B       .
                ENDP

BTIMR0_Handler  PROC
                EXPORT  BTIMR0_Handler             [WEAK]
                B       .
                ENDP

BTIMR1_Handler  PROC
                EXPORT  BTIMR1_Handler             [WEAK]
                B       .
                ENDP

BTIMR2_Handler  PROC
                EXPORT  BTIMR2_Handler             [WEAK]
                B       .
                ENDP

BTIMR3_Handler  PROC
                EXPORT  BTIMR3_Handler             [WEAK]
                B       .
                ENDP

GPIOA_Handler  PROC
                EXPORT  GPIOA_Handler              [WEAK]
                B       .
                ENDP

GPIOB_Handler  PROC
                EXPORT  GPIOB_Handler              [WEAK]
                B       .
                ENDP

GPIOM_Handler  PROC
                EXPORT  GPIOM_Handler              [WEAK]
                B       .
                ENDP

GPIOA0_GPIOM0_SPI1_Handler  PROC
                EXPORT  GPIOA0_GPIOM0_SPI1_Handler [WEAK]
                B       .
                ENDP

GPIOA1_GPIOM1_MPU_Handler  PROC
                EXPORT  GPIOA1_GPIOM1_MPU_Handler  [WEAK]
                B       .
                ENDP

GPIOA2_GPIOM2_Handler  PROC
                EXPORT  GPIOA2_GPIOM2_Handler      [WEAK]
                B       .
                ENDP

GPIOA3_GPIOM3_Handler  PROC
                EXPORT  GPIOA3_GPIOM3_Handler      [WEAK]
                B       .
                ENDP

GPIOB0_GPIOA8_TIMR2_Handler  PROC
                EXPORT  GPIOB0_GPIOA8_TIMR2_Handler [WEAK]
                B       .
                ENDP

GPIOB1_GPIOA9_DMA_Handler  PROC
                EXPORT  GPIOB1_GPIOA9_DMA_Handler   [WEAK]
                B       .
                ENDP

GPIOB2_GPIOA10_CAN0_Handler  PROC
                EXPORT  GPIOB2_GPIOA10_CAN0_Handler [WEAK]
                B       .
                ENDP

GPIOB3_GPIOA11_SPI0_Handler  PROC
                EXPORT  GPIOB3_GPIOA11_SPI0_Handler [WEAK]
                B       .
                ENDP

GPIOB4_GPIOB10_QEI_Handler  PROC
                EXPORT  GPIOB4_GPIOB10_QEI_Handler  [WEAK]
                B       .
                ENDP

                ALIGN


;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF
    
                END
