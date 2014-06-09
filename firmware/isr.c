extern void main ();
void _errorloop () __attribute__ ((noreturn));

void Reset_Handler ();
extern void NMI_Handle ();
extern void HardFault_Handler ();
extern void MemManage_Handler ();
extern void BusFault_Handler ();
extern void UsageFault_Handler ();
extern void SVCall_Handler ();
extern void DebugMonitor_Handler ();
extern void PendSV_Handler ();
extern void SysTick_Handler ();

extern void WWDG_Handler ();
extern void PVD_Handler ();
extern void TAMPER_Handler ();
extern void RTC_Handler ();
extern void FLASH_Handler ();
extern void RCC_Handler ();
extern void EXTI0_Handler ();
extern void EXTI1_Handler ();
extern void EXTI2_Handler ();
extern void EXTI3_Handler ();
extern void EXTI4_Handler ();
extern void DMA1_Channel1_Handler ();
extern void DMA1_Channel2_Handler ();
extern void DMA1_Channel3_Handler ();
extern void DMA1_Channel4_Handler ();
extern void DMA1_Channel5_Handler ();
extern void DMA1_Channel6_Handler ();
extern void DMA1_Channel7_Handler ();
extern void ADC1_2_Handler ();
extern void USB_HP_CAN_TX_Handler ();
extern void USB_LP_CAN_RX0_Handler ();
extern void CAN_RX1_Handler ();
extern void CAN_SCE_Handler ();
extern void EXTI9_5_Handler ();
extern void TIM1_BRK_Handler ();
extern void TIM1_UP_Handler ();
extern void TIM1_TRG_COM_Handler ();
extern void TIM1_CC_Handler ();
extern void TIM2_Handler ();
extern void TIM3_Handler ();
extern void TIM4_Handler ();
extern void I2C1_EV_Handler ();
extern void I2C1_ER_Handler ();
extern void I2C2_EV_Handler ();
extern void I2C2_ER_Handler ();
extern void SPI1_Handler ();
extern void SPI2_Handler ();
extern void USART1_Handler ();
extern void USART2_Handler ();
extern void USART3_Handler ();
extern void EXTI15_10_Handler ();
extern void RTCAlarm_Handler ();
extern void USBWakeup_Handler ();
extern void TIM8_BRK_Handler ();
extern void TIM8_UP_Handler ();
extern void TIM8_TRG_COM_Handler ();
extern void TIM8_CC_Handler ();
extern void ADC3_Handler ();
extern void FSMC_Handler ();
extern void SDIO_Handler ();
extern void TIM5_Handler ();
extern void SPI3_Handler ();
extern void UART4_Handler ();
extern void UART5_Handler ();
extern void TIM6_Handler ();
extern void TIM7_Handler ();
extern void DMA2_Channel1_Handler ();
extern void DMA2_Channel2_Handler ();
extern void DMA2_Channel3_Handler ();
extern void DMA2_Channel4_5_Handler ();

typedef void (*pfnISR)(void);

__attribute__ ((section(".isr_vector")))
pfnISR VectorTable[] =  
{ 
  (pfnISR)(0x20000000 + RAMSIZE), // The initial stack pointer is the top of SRAM 
  Reset_Handler,
  NMI_Handle, 
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler,
  UsageFault_Handler,
  0,
  0,
  0,
  0,
  SVCall_Handler,
  DebugMonitor_Handler,
  0,
  PendSV_Handler,
  SysTick_Handler,

	/*  0 */ 0, // WWDG_Handler,
	/*  1 */ 0, // PVD_Handler,
	/*  2 */ 0, // TAMPER_Handler,
	/*  3 */ 0, // RTC_Handler,
	/*  4 */ 0, // FLASH_Handler,
	/*  5 */ 0, // RCC_Handler,
	/*  6 */ 0, // EXTI0_Handler,
	/*  7 */ 0, // EXTI1_Handler,
	/*  8 */ 0, // EXTI2_Handler,
	/*  9 */ 0, // EXTI3_Handler,
	/* 10 */ 0, // EXTI4_Handler,
	/* 11 */ 0, // DMA1_Channel1_Handler,
	/* 12 */ 0, // DMA1_Channel2_Handler,
	/* 13 */ 0, // DMA1_Channel3_Handler,
	/* 14 */ 0, // DMA1_Channel4_Handler,
	/* 15 */ 0, // DMA1_Channel5_Handler,
	/* 16 */ 0, // DMA1_Channel6_Handler,
	/* 17 */ 0, // DMA1_Channel7_Handler,
	/* 18 */ 0, // ADC1_2_Handler,
	/* 19 */ 0, // USB_HP_CAN_TX_Handler,
	/* 20 */ USB_LP_CAN_RX0_Handler,
	/* 21 */ 0, // CAN_RX1_Handler,
	/* 22 */ 0, // CAN_SCE_Handler,
	/* 23 */ 0, // EXTI9_5_Handler,
	/* 24 */ 0, // TIM1_BRK_Handler,
	/* 25 */ 0, // TIM1_UP_Handler,
	/* 26 */ 0, // TIM1_TRG_COM_Handler,
	/* 27 */ 0, // TIM1_CC_Handler,
	/* 28 */ 0, // TIM2_Handler,
	/* 29 */ 0, // TIM3_Handler,
	/* 30 */ 0, // TIM4_Handler,
	/* 31 */ 0, // I2C1_EV_Handler,
	/* 32 */ 0, // I2C1_ER_Handler,
	/* 33 */ 0, // I2C2_EV_Handler,
	/* 34 */ 0, // I2C2_ER_Handler,
	/* 35 */ 0, // SPI1_Handler,
	/* 36 */ 0, // SPI2_Handler,
	/* 37 */ USART1_Handler,
	/* 38 */ 0, // USART2_Handler,
	/* 39 */ 0, // USART3_Handler,
	/* 40 */ 0, // EXTI15_10_Handler,
	/* 41 */ 0, // RTCAlarm_Handler,
	/* 42 */ 0, // USBWakeup_Handler,
	/* 43 */ 0, // TIM8_BRK_Handler,
	/* 44 */ 0, // TIM8_UP_Handler,
	/* 45 */ 0, // TIM8_TRG_COM_Handler,
	/* 46 */ 0, // TIM8_CC_Handler,
	/* 47 */ 0, // ADC3_Handler,
	/* 48 */ 0, // FSMC_Handler,
	/* 49 */ 0, // SDIO_Handler,
	/* 50 */ 0, // TIM5_Handler,
	/* 51 */ 0, // SPI3_Handler,
	/* 52 */ 0, // UART4_Handler,
	/* 53 */ 0, // UART5_Handler,
	/* 54 */ 0, // TIM6_Handler,
	/* 55 */ 0, // TIM7_Handler,
	/* 56 */ 0, // DMA2_Channel1_Handler,
	/* 57 */ 0, // DMA2_Channel2_Handler,
	/* 58 */ 0, // DMA2_Channel3_Handler,
	/* 59 */ 0, // DMA2_Channel4_5_Handler,
};

extern unsigned long _eisr_vector;
extern unsigned long _text;
extern unsigned long _etext; 
extern unsigned long _data; 
extern unsigned long _edata; 
extern unsigned long _bss; 
extern unsigned long _ebss; 
extern unsigned long _sidata; 
extern unsigned long _sdata; 

void Reset_Handler () 
{ 
  unsigned long *src, *dst; 

  src = &_sidata;
  dst = &_sdata;
  while (dst < &_edata)
    *dst++ = *src++;

	dst = &_bss;
	while (dst < &_ebss)
    *dst++ = 0; 

  main ();
} 
void NMI_Handle () { _errorloop (); }
void HardFault_Handler () { _errorloop (); }
void MemManage_Handler () { _errorloop (); }
void BusFault_Handler () { _errorloop (); }
void UsageFault_Handler () { _errorloop (); }
void SVCall_Handler () { _errorloop (); }
void DebugMonitor_Handler () { _errorloop (); }
void PendSV_Handler () { _errorloop (); }
// void SysTick_Handler () { _errorloop (); }

// void WWDG_Handler () { _errorloop (); }
// void PVD_Handler () { _errorloop (); }
// void TAMPER_Handler () { _errorloop (); }
// void RTC_Handler () { _errorloop (); }
// void FLASH_Handler () { _errorloop (); }
// void RCC_Handler () { _errorloop (); }
// void EXTI0_Handler () { _errorloop (); }
// void EXTI1_Handler () { _errorloop (); }
// void EXTI2_Handler () { _errorloop (); }
// void EXTI3_Handler () { _errorloop (); }
// void EXTI4_Handler () { _errorloop (); }
// void DMA1_Channel1_Handler () { _errorloop (); }
// void DMA1_Channel2_Handler () { _errorloop (); }
// void DMA1_Channel3_Handler () { _errorloop (); }
// void DMA1_Channel4_Handler () { _errorloop (); }
// void DMA1_Channel5_Handler () { _errorloop (); }
// void DMA1_Channel6_Handler () { _errorloop (); }
// void DMA1_Channel7_Handler () { _errorloop (); }
// void ADC1_2_Handler () { _errorloop (); }
// void USB_HP_CAN_TX_Handler () { _errorloop (); }
// void USB_LP_CAN_RX0_Handler () { _errorloop (); }
// void CAN_RX1_Handler () { _errorloop (); }
// void CAN_SCE_Handler () { _errorloop (); }
// void EXTI9_5_Handler () { _errorloop (); }
// void TIM1_BRK_Handler () { _errorloop (); }
// void TIM1_UP_Handler () { _errorloop (); }
// void TIM1_TRG_COM_Handler () { _errorloop (); }
// void TIM1_CC_Handler () { _errorloop (); }
// void TIM2_Handler () { _errorloop (); }
// void TIM3_Handler () { _errorloop (); }
// void TIM4_Handler () { _errorloop (); }
// void I2C1_EV_Handler () { _errorloop (); }
// void I2C1_ER_Handler () { _errorloop (); }
// void I2C2_EV_Handler () { _errorloop (); }
// void I2C2_ER_Handler () { _errorloop (); }
// void SPI1_Handler () { _errorloop (); }
// void SPI2_Handler () { _errorloop (); }
// void USART2_Handler () { _errorloop (); }
// void USART3_Handler () { _errorloop (); }
// void EXTI15_10_Handler () { _errorloop (); }
// void RTCAlarm_Handler () { _errorloop (); }
// void USBWakeup_Handler () { _errorloop (); }
// void TIM8_BRK_Handler () { _errorloop (); }
// void TIM8_UP_Handler () { _errorloop (); }
// void TIM8_TRG_COM_Handler () { _errorloop (); }
// void TIM8_CC_Handler () { _errorloop (); }
// void ADC3_Handler () { _errorloop (); }
// void FSMC_Handler () { _errorloop (); }
// void SDIO_Handler () { _errorloop (); }
// void TIM5_Handler () { _errorloop (); }
// void SPI3_Handler () { _errorloop (); }
// void UART4_Handler () { _errorloop (); }
// void UART5_Handler () { _errorloop (); }
// void TIM6_Handler () { _errorloop (); }
// void TIM7_Handler () { _errorloop (); }
// void DMA2_Channel1_Handler () { _errorloop (); }
// void DMA2_Channel2_Handler () { _errorloop (); }
// void DMA2_Channel3_Handler () { _errorloop (); }
// void DMA2_Channel4_5_Handler () { _errorloop (); }
