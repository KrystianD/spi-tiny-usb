#include <public.h>
#include <hardware.h>
#include <settings.h>
#include <delay.h>
#include <myprintf.h>
#include <kdusb.h>

volatile uint32_t ticks = 0;

void myputchar(int c)
{
	USART1->DR = c;
	while (!(USART1->SR & USART_SR_TC));
}

void main()
{
	int i;
	
	SCB->VTOR = 0x08000000;
	
	// // Setup wait-states
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV8 | RCC_CFGR_USBPRE;
	
	// // Enable HSE
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));
	
	// // Setup PLL x6 (48MHz) as clock source
	RCC->CFGR |= RCC_CFGR_PLLMULL6 | /*RCC_CFGR_PLLXTPRE |*/ RCC_CFGR_PLLSRC;
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	
	// Enable peripherals
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_USBEN;
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
	
	// Enable systick
	SysTick->LOAD = SysTick->VAL = (F_CPU / 1000) / 8;
	SysTick->CTRL = /*SysTick_CTRL_CLKSOURCE |*/ SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT;
	
	// Enable USART
	IO_ALT_PUSH_PULL(UART_TX);
	USART1->BRR = USART_BRR(115200);
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
	ENABLE_INTERRUPT(USART1_IRQn);
	
	// Enable SPI
	RCC->SPI_SPI_APB |= SPI_SPI_APBEN;
	SPI_SPI->CR1 = (SPI_DIV) | SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
	IO_PUSH_PULL(SPI_CE);
	IO_ALT_PUSH_PULL(SPI_SCK);
	IO_ALT_PUSH_PULL(SPI_MOSI);
	IO_INPUT_PP(SPI_MISO);
	IO_HIGH(SPI_MISO);
	IO_HIGH(SPI_CE);
	
	_delay_init();
	IO_PUSH_PULL(LED);
	
	// Init USB
	usbInit();
	ENABLE_INTERRUPT(USB_LP_CAN1_RX0_IRQn);
	usbDisconnect();
	_delay_ms(100);
	usbConnect();
	
	for (;;)
	{
	}
}
void _errorloop()
{
	while (1)
	{
		// IO_TOGGLE(led);
		_delay_ms(100);
	}
}
void SysTick_Handler()
{
	ticks++;
}


void enableSPI()
{
	IO_LOW(SPI_CE);
}
void disableSPI()
{
	IO_HIGH(SPI_CE);
}
uint8_t SPI_RW(uint8_t val)
{
	SPI_SPI->DR = val;
	while (!(SPI_SPI->SR & SPI_SR_TXE));
	while (!(SPI_SPI->SR & SPI_SR_RXNE));
	return SPI_SPI->DR;
}

uint8_t data[4 * 1024];
uint8_t inData[64];
volatile int total = 0;
uint16_t usbFunctionSetup()
{
	uint8_t reg, len, i;
	
	myprintf("%d %d %d %d\r\n",
	         usbRequest.bRequest,
	         usbRequest.wValue.word,
	         usbRequest.wIndex.word,
	         usbRequest.wLength);
	         
	switch (usbRequest.bRequest)
	{
	case 0:
		usbData = inData;
		total = 0;
		return 0;
	case 1:
		usbData = data;
		return usbRequest.wLength;
	}
}
void usbHandleData(uint8_t size)
{
	uint8_t reg, len, i;
	
	switch (usbRequest.bRequest)
	{
	case 0:
	
		enableSPI();
		for (i = 0; i < size; i++)
		{
			uint8_t d = SPI_RW(inData[i]);
			data[total++] = d;
		}
		disableSPI();
		
		myprintf("new pac2: %d %d\r\n", size, total);
		break;
	}
}

void USART1_Handler()
{
	if (USART1->SR & USART_SR_RXNE)
	{
		uint8_t d = USART1->DR;
		if (d == 0x7f)
		{
			*((unsigned long*)0x0E000ED0C) = 0x05FA0004;
			while (1);
		}
	}
}
