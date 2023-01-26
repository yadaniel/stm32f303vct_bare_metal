#include <stdint.h>
#include <stdbool.h>
/* #include <string.h> */
#include "user.h"

// stm32f3 discovery board with stm32f303vct

#define __IO    volatile

typedef struct {
  __IO uint32_t CR;         /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
} RCC_TypeDef;

typedef struct {
  __IO uint32_t MODER;        /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,               Address offset: 0x28 */
} GPIO_TypeDef;

typedef struct {
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED1;  /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED2;  /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;

#define RCC     ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x00001000UL))
#define GPIOE   ((GPIO_TypeDef *) ((0x40000000UL + 0x08000000UL) + 0x00001000UL))
#define USART1  ((USART_TypeDef *) ((0x40000000UL + 0x00010000UL) + 0x00003800UL))

#define USART_ISR_TXE   (1UL << 7)

 uint32_t main_version = 0xABCD;
//uint32_t main_version = 0xABCE;
uint32_t main_counter = 0;
//uint32_t main_counter = 1;

// refman
extern void ResetHandler(void);
extern void Default_Handler(void);
extern void NMI_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Hardfault_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Memfault_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Busfault_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Usagefault_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void SVC_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Debugmon_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Pending_Handler(void) __attribute__((weak,alias("Default_Handler")));
extern void Systick_Handler(void) __attribute__((weak,alias("Default_Handler")));

// note: 40kB normal SRAM, 8kB CCM => hardfault when using SRAM_SIZE = 48kB
#define SRAM_START      0x20000000U
#define SRAM_SIZE       (40U*1024U)
#define SRAM_END        ((SRAM_START) + (SRAM_SIZE))
#define STACK_START     SRAM_END

// use KEEP in linker script, otherwise --gc-sections will not include it
static uint32_t vector[98] __attribute__((section(".isr_vector"))) = {
    STACK_START,
    (uint32_t)&ResetHandler,
    (uint32_t)&NMI_Handler,
    (uint32_t)&Hardfault_Handler,
    (uint32_t)&Memfault_Handler,
    (uint32_t)&Busfault_Handler,
    (uint32_t)&Usagefault_Handler,
    0,
    0,
    0,
    0,
    (uint32_t)&SVC_Handler,
    (uint32_t)&Debugmon_Handler,
    0,
    (uint32_t)&Pending_Handler,
    (uint32_t)&Systick_Handler,
    //

};

// symbols added by linker script
// within c context the content of 'variables' is not relevant
// within c context only the address of 'variables' is relevant
// compiler and linker symbol table can have more that one symbol for given address => overlay
// because of overlay 'variables' do not consume RAM, they happen to overlay other user defined variables
//
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
//
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __la_data__;

// provided by newlib-nano
extern void __libc_init_array(void);
extern int main(void);

void Default_Handler(void) {

    RCC->AHBENR = 0xFFFFFFFF;
    RCC->APB1ENR = 0xFFFFFFFF;
    RCC->APB2ENR = 0xFFFFFFFF;

    RCC->AHBENR |= 1 << 21;
    GPIOE->MODER |= 1 << 30;    // 15*2
    while(1) {

        GPIOE->ODR |= (1UL << 15);

        /* volatile uint8_t i = 1; */
        /* while(i>2) {} */
        /* while(false) {} */

        // when using 48kB RAM -> hardfault entered recursively

        /* GPIOE->ODR &= ~(1UL << 15); */
        /* for(volatile uint32_t i=0; i<0xFF; i++) { */
        /* } */

        /* while(1); */
    }

}

void ResetHandler(void) {

    // SystemInit
    // SCB = system control block
    // SBC->CPACR co-processor access control register
    // SCB->CPACR |= ((3UL << 20)|(3UL << 22));

    uint32_t size;
    uint8_t * pSrc;
    uint8_t * pDst;

    // copy data
    size = ((uint32_t)&__data_end__) - ((uint32_t)&__data_start__);
    pDst = (uint8_t *)&__data_start__;
    pSrc = (uint8_t *)&__la_data__;
    for(uint32_t i=0; i<size; i++) {
        *pDst = *pSrc;
        pDst += 1;
        pSrc += 1;
    }

    // zero out bss
    size = ((uint32_t)&__bss_end__) - ((uint32_t)&__bss_start__);
    pDst = (uint8_t *)&__bss_start__;
    for(uint32_t i=0; i<size; i++) {
        *pDst = 0;
        pDst += 1;
    }

    // call newlib-nano initialization
    /* __libc_init_array(); */

    // call main
    main();

    // when main returns => block
    // alternative reset controller
    while(1) {
    }

}

void delay(void) {

    volatile uint32_t i = 0xFFFF;
    volatile uint32_t j;
    while(i!=0) {
        i-=1;

        /* j = 0xFF; */
        /* while(j!=0) { */
        /*     j -= 1; */
        /* } */

    }

}

int main(void) {

    uint32_t len = 0;
    //uint32_t len = strlen("foo");

    /* RCC->AHBENR |= 1 << 21; */
    /* GPIOE->MODER |= 1 << 30; */
    /* GPIOE->ODR |= 1 << 15; */

    RCC->AHBENR = 0xFFFFFFFF;
    RCC->APB1ENR = 0xFFFFFFFF;
    RCC->APB2ENR = 0xFFFFFFFF;

    /* RCC->CR = 1; */
    /* while((RCC->CR & 2) != 2) {} */

    /* RCC->CR = 1<<24; */
    /* while((RCC->CR & (1<<24)) != (1<<24)) {} */

    RCC->AHBENR |= 1 << 21;
    GPIOE->MODER |= 1 << 30;    // 15*2
    GPIOE->MODER |= 1 << 22;    // 11*2
    GPIOE->MODER |= 1 << 18;    // 9*2

    /* USART1->CR1 |= 8 | 1; */

    while(1) {

        /* while(!(USART1->ISR & USART_ISR_TXE)); // block until tx empty */
        /* USART1->TDR = 'B'; */

        delay();
        GPIOE->ODR |= (1UL << 15);
        if(main_version == 0xABCD) {
            // checks if data section initialized
            GPIOE->ODR |= (1UL << 11);
        }
        if(main_counter == 0) {
            // checks if bss section initialized
            GPIOE->ODR |= (1UL << 9);
        }

        delay();
        GPIOE->ODR &= ~(1UL << 15);
        if(main_version == 0xABCD) {
            GPIOE->ODR &= ~(1UL << 11);
        }
        if(main_counter == 0) {
            GPIOE->ODR &= ~(1UL << 9);
        }

        /* USART1->TDR = 'B'; */
        /* while(!(USART1->ISR & USART_ISR_TXE)); // block until tx empty */

    }

    return 0;
}

