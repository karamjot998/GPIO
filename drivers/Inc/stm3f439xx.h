#ifndef INC_STM3F439XX_H_
#define INC_STM3F439XX_H_

#include <stdint.h>

#define _vo volatile
/*
 * base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM						SRAM1_BASEADDR

/*
 * base addresses oh AHBx and APBx
 */
#define PERPIH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U


/*
 * base addresses of peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASE + 0x2800)

#define RCC_BASEADDR				(AHB1PERIPH_BASE +  0X3800)



/*
 *  RCC PERIPHERAL REGISTER DEFINITION STRUCTURE
 */
typedef struct {
	volatile uint32_t CR;            // 0x00 Clock control register
	volatile uint32_t PLLCFGR;       // 0x04 PLL configuration register
	volatile uint32_t CFGR;          // 0x08 Clock configuration register
	volatile uint32_t CIR;           // 0x0C Clock interrupt register

	volatile uint32_t AHB1RSTR;      // 0x10 AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;      // 0x14 AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;      // 0x18 AHB3 peripheral reset register
	uint32_t RESERVED0;              // 0x1C Reserved

	volatile uint32_t APB1RSTR;      // 0x20 APB1 peripheral reset register
	volatile uint32_t APB2RSTR;      // 0x24 APB2 peripheral reset register
	uint32_t RESERVED1[2];           // 0x28–0x2C Reserved

	volatile uint32_t AHB1ENR;       // 0x30 AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;       // 0x34 AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;       // 0x38 AHB3 peripheral clock enable register
	uint32_t RESERVED2;              // 0x3C Reserved

	volatile uint32_t APB1ENR;       // 0x40 APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;       // 0x44 APB2 peripheral clock enable register
	uint32_t RESERVED3[2];           // 0x48–0x4C Reserved

	volatile uint32_t AHB1LPENR;     // 0x50 AHB1 low power clock enable register
	volatile uint32_t AHB2LPENR;     // 0x54 AHB2 low power clock enable register
	volatile uint32_t AHB3LPENR;     // 0x58 AHB3 low power clock enable register
	uint32_t RESERVED4;              // 0x5C Reserved

	volatile uint32_t APB1LPENR;     // 0x60 APB1 low power clock enable register
	volatile uint32_t APB2LPENR;     // 0x64 APB2 low power clock enable register
	uint32_t RESERVED5[2];           // 0x68–0x6C Reserved

	volatile uint32_t BDCR;          // 0x70 Backup domain control register
	volatile uint32_t CSR;           // 0x74 Clock control & status register
	uint32_t RESERVED6[2];           // 0x78–0x7C Reserved

	volatile uint32_t SSCGR;         // 0x80 Spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;    // 0x84 PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;    // 0x88 PLLSAI configuration register
	volatile uint32_t DCKCFGR;       // 0x8C Dedicated clocks configuration register
	volatile uint32_t CKGATENR;      // 0x90 Clocks gated enable register
	volatile uint32_t DCKCFGR2;      // 0x94 Dedicated clocks configuration register 2

}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)
/*
 *GPIO PERIPHERAL REGISTER DEFINATION STRUCTURE
 */

typedef struct {
	_vo uint32_t MODER;				// GPIO port mode register (GPIOx_MODER)
	_vo uint32_t OTYPER;			// GPIO port output type register (GPIOx_OTYPER
	_vo uint32_t OSPEEDR;			// GPIO port output speed register (GPIOx_OSPEEDR)
	_vo uint32_t PUPDR;				// GPIO port pull-up/pull-down register (GPIOx_PUPDR)
	_vo uint32_t  IDR;				// GPIO port input data register (GPIOx_IDR) (x = A..I/J/K)
	_vo uint32_t ODR;				// GPIO port output data register (GPIOx_ODR) (x = A..I/J/K)
	_vo uint32_t BSRR;				// GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K)
	_vo uint32_t LCKR;				// GPIO port configuration lock register (GPIOx_LCKR)
	_vo uint32_t AFR[2];			// GPIO alternate function low register (GPIOx_AFR[0] = low register, GPIOx_AFR[1] = High register)
}GPIO_RegDef_t;


/*
 * peripheral definitions ( peripheral base addressse typecasted to xx_RegDef_t)
 */
#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*) GPIOK_BASEADDR)


/*
 * GPIO clock enable macros
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 8 ))
#define GPIOJ_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 9 ))
#define GPIOK_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 10 ))


/*
 * I2C CLOCK ENABLE MACROS
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 23))

/*
 * SPI CLOCK ENABKE MACRO
 */

/*
 * USART CLOCK ENABLE MACRO
 */


/*
 * CLOCK DISABLE MACRO FOR GPIO
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 8 ))
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 9 ))
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 10 ))


/*
 * CLOCK DISABLE MACRO FOR I2C
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 23))



/*
 * CLOCK DISABLE MACRO FOR SPI
 */

/*
 * CLOCK DISABLE MACRO FOR USART
 */


#endif
