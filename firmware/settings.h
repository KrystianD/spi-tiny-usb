
#define DELAY_TIMER TIM3
#define DELAY_TIMER_APB APB1ENR
#define DELAY_TIMER_APBEN RCC_APB1ENR_TIM3EN

#define SPI_SPI       SPI1
#define SPI_SPI_APB   APB2ENR
#define SPI_SPI_APBEN RCC_APB2ENR_SPI1EN;
#define SPI_DIV      SPI_CR1_BR_2 
//(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

#define MAIN_DEBUG(x,...) myprintf("[] " x "\r\n", ##__VA_ARGS__)
#define RF_DEBUG(x,...) myprintf("[RF] " x "\r\n", ##__VA_ARGS__)
// #define KDRF_DEBUG(x,...) myprintf("[KDRF] " x "\r\n", ##__VA_ARGS__)

#define KDRF_TYPICAL_SEND_TIME 20
