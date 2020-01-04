// JAZ
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file                     : main.c
 * @brief                    : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                                                opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

// a struct that defines a GPIO device (maps GPIOs memory structure)
typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint16_t BSR; // write only
	uint16_t BRR; // write only
} GPIO_device;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

// constants for storing addresses of GPIOs
#define GPIOAd ((GPIO_device *)0x40020000)
#define GPIOBd ((GPIO_device *)0x40020400)
#define GPIOCd ((GPIO_device *)0x40020800)
#define GPIODd ((GPIO_device *)0x40020C00)
#define GPIOEd ((GPIO_device *)0x40021000)
#define GPIOFd ((GPIO_device *)0x40021400)
#define GPIOGd ((GPIO_device *)0x40021800)
#define GPIOHd ((GPIO_device *)0x40021C00)
#define GPIOId ((GPIO_device *)0x40022000)

#define RCC_AHB1ENR ((uint32_t *)0x40023830)

// MODE constants; odmik: 0x00
#define IN 0x00
#define OUT 0x01
#define AF 0x11
#define ANALOG 0x10

// PUPD constants; odmik: 0x0C
#define NO_PULL 0x00
#define PULL_UP 0x01
#define PULL_DOWN 0x10

// OTYPE constants; odmik: 0x40
#define PUSH_PULL 0x00
#define OPEN_DRAIN 0x01

// OSPEED constants; odmik: 0x08
#define S2MHz 0x00
#define S25MHz 0x01
#define S50MHz 0x10
#define S100MHz 0x11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void clock_on(GPIO_device *GPIO_addr);
void init_GPIO(GPIO_device *GPIO_addr, uint32_t Pin, uint32_t Mode,
			   uint32_t PUPD, uint32_t OType, uint32_t OSpeed);
void GPIO_pin_write(GPIO_device *GPIO_addr, uint32_t Pin, uint32_t val);
uint32_t GPIO_pin_read(GPIO_device *GPIO_addr, uint32_t Pin);
void delay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief    The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */

	// turn on button's GPIO (button is on PA0)
	clock_on(GPIOAd);

	// turn on led's GPIO (leds are on PD12, PD13, PD14, PD15)
	clock_on(GPIODd);

	// init button
	init_GPIO(GPIOAd, 0, IN, NO_PULL, PUSH_PULL, S2MHz);

	// init leds
	for (int i = 12; i < 16; i++)
	{
		init_GPIO(GPIODd, i, OUT, NO_PULL, PUSH_PULL, S2MHz);
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/*
		 GPIO_pin_write(GPIODd, 12, 1);
		 delay();
		 GPIO_pin_write(GPIODd, 13, 1);
		 delay();
		 GPIO_pin_write(GPIODd, 14, 1);
		 delay();
		 GPIO_pin_write(GPIODd, 15, 1);
		 delay();
		 delay();
		 delay();
		 delay();
		 delay();
		 */
		// read button and trigger led sequence
		if (GPIO_pin_read(GPIOAd, 0) == 1)
		{ // preberemo gumb
			for (int i = 12; i < 16; i++)
			{ // gremi čez lučke in jih prižgemo z zamikom
				GPIO_pin_write(GPIODd, i, 1);
				delay();
			}
			for (int i = 12; i < 16; i++)
			{ // ugasnemo vse naenkrat
				GPIO_pin_write(GPIODd, i, 0);
			}

			for (int i = 0; i < 3; i++)
			{

				GPIO_pin_write(GPIODd, 12, 1);
				GPIO_pin_write(GPIODd, 14, 1);
				delay();
				GPIO_pin_write(GPIODd, 14, 0);
				GPIO_pin_write(GPIODd, 12, 0);
				delay();
				GPIO_pin_write(GPIODd, 12, 1);
				GPIO_pin_write(GPIODd, 14, 1);
				delay();
				GPIO_pin_write(GPIODd, 14, 0);
				GPIO_pin_write(GPIODd, 12, 0);
				delay();

				GPIO_pin_write(GPIODd, 13, 1);
				GPIO_pin_write(GPIODd, 15, 1);
				delay();
				GPIO_pin_write(GPIODd, 13, 0);
				GPIO_pin_write(GPIODd, 15, 0);
				delay();
				GPIO_pin_write(GPIODd, 13, 1);
				GPIO_pin_write(GPIODd, 15, 1);
				delay();
				GPIO_pin_write(GPIODd, 13, 0);
				GPIO_pin_write(GPIODd, 15, 0);
				delay();
			}

			for (int i = 15; i > 11; i--)
			{ // gremi čez lučke in jih prižgemo z zamikom
				GPIO_pin_write(GPIODd, i, 1);
				delay();
			}
			for (int i = 12; i < 16; i++)
			{ // ugasnemo vse naenkrat
				GPIO_pin_write(GPIODd, i, 0);
			}

			for (int x = 0; x < 4; x++)
			{
				for (int i = 12; i < 16; i++)
				{ // gremi čez lučke in jih prižgemo z zamikom
					GPIO_pin_write(GPIODd, i, 1);
				}
				for (int i = 12; i < 16; i++)
				{ // gremi čez lučke in jih prižgemo z zamikom
					GPIO_pin_write(GPIODd, i, 0);
				}
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void resetBit(volatile uint32_t *address, uint8_t p)
{
	*address = *address & ~(1 << p);
}

void resetTwoBits(volatile uint32_t *address, uint8_t p)
{
	// reset two bits (p and p+1) in a 32 bit register
	*address = *address & ~(1 << p);
	*address = *address & ~(1 << (p + 1));
}

void setBit(volatile uint32_t *address, uint8_t p)
{
	// set bit p in a 32 bit register
	*address = *address | (1 << p);
}

void setBit16(volatile uint16_t *address, uint8_t p)
{
	// set bit p in a 16 bit register
	*address = *address | (1 << p);
}

void setTwoBitsTo(volatile uint32_t *address, uint8_t p, uint8_t n)
{
	// set two bits (p and p+1) to value n in a 32 bit register
	*address = *address & ~(1 << p);
	*address = *address & ~(1 << (p + 1));

	if (n == 0)
	{
		// p-ti bit na 00
	}
	else if (n == 1)
	{
		// p-ti bit na 01
		*address = *address | (1 << p);
	}
	else if (n == 2)
	{
		// p-ti bit na 10
		*address = *address | (1 << p);
	}
	else if (n == 3)
	{
		// p-ti bit na 11
		*address = *address | (1 << p);
		*address = *address | (1 << (p + 1));
	}
}

// function for turning on a particular GPIO (enables the port clock)
void clock_on(GPIO_device *GPIO_addr)
{

	// *RCC_AHB1ENR |= (1UL << (((unsigned long)GPIO_addr-(unsigned long)GPIOAd)/0x400));
	// uint32_t* clockAdress = (uint32_t*) 0x40023830;

	// dobimo int naprave, za toliko shiftamo bit v levo da ji prižgemo uro
	// 0x4002 0000 0100 0000 0000
	long naprava = ((long)GPIO_addr - (long)GPIOAd) / 0x400;
	*RCC_AHB1ENR = *RCC_AHB1ENR | (1 << naprava);

	// int num = naprava / 0x400;
	// setBit((uint32_t*) clockAdress, num);
	// *uint32_t = *uint32_t | (1 << num);
}

// function for initializing GPIOs
void init_GPIO(GPIO_device *GPIO_addr, uint32_t Pin, uint32_t Mode,
			   uint32_t PUPD, uint32_t OType, uint32_t OSpeed)
{

	// MODER oz način delovanja; po dva bita gre od 0 in 1 do 30 in 31 bita, ki nastavljata pin 15
	// bita 00 - pin deluje kot vhod
	// bita 01 - pin izhod
	// ( 10 - analogni način )
	// ( 11 - način alternativne funkcije )
	// GPIO_addr->MODER = (GPIO_addr->MODER & ~(3 << Pin*2))|(Mode << Pin*2);

	GPIO_addr->MODER &= ~(0b11 << Pin * 2); // pobrišemo bite
	GPIO_addr->MODER |= (Mode << Pin * 2);  // nastavimo bite, odvisno od moda
	//GPIO_addr->MODER = (GPIO_addr->MODER & ~(3 << Pin*2));

	// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	// PUPDR, dva bita namenjena posameznemu pinu. Če uporabljamo pin kot vhod ALI izhod
	// 00 - /   (nočemo niti pull-up niti pull-down upora)
	// 01 - pull-up
	// 10 - pull-down
	// 11 - se ne uporablja (prepovedano)
	// Odmik : 0x0C
	// zaradi led_port_clock_on() iz prejšnjih vaj za 3 premaknemo

	//GPIO_addr->PUPDR = (GPIO_addr->PUPDR & ~(3 << Pin*2));

	GPIO_addr->PUPDR &= ~(0b11 << Pin * 2);
	GPIO_addr->PUPDR |= (PUPD << Pin * 2);

	// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	// OSPEEDR, hitrost osveževanja samo če uporabljamo kot izhod
	// S2MHz == 0x00; zadostuje najnižja hitrost

	//GPIO_addr->OSPEEDR = (GPIO_addr->OSPEEDR & ~(3 << Pin*2))|(OSpeed << Pin*2);

	GPIO_addr->OSPEEDR &= ~(0b11 << Pin * 2);
	GPIO_addr->OSPEEDR |= (OSpeed << Pin * 2);

	//resetTwoBits(GPIO_addr->OSPEEDR, (Pin*2));

	// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	// OTYPER, način izhoda, samo če uporabljamo kot izhod
	// odmik: 0x40
	// PUSH_PULL == 0x01  -- vedno na vajah!
	// OPEN_DRAIN == 0x00
	// 32 biten register, uporablja se spodnjih 16 bitov. bit 0, za pin 0, bit 1 za pin 1...

	//GPIO_addr->OTYPER  = (GPIO_addr->OTYPER & ~(1UL << Pin))|(OType << Pin);

	GPIO_addr->OTYPER &= ~(1 << Pin);
	GPIO_addr->OTYPER |= (OType << Pin);

	// setBit(GPIO_addr->OTYPER, Pin);
}

// function for setting the value of an output GPIO pin
void GPIO_pin_write(GPIO_device *GPIO_addr, uint32_t Pin, uint32_t val)
{

	if (val == 1)
	{
		// BSR - bit set - odmik 0x18
		GPIO_addr->BSR = 1 << Pin;
		//setBit(GPIO_addr->BSR, Pin);
		//GPIO_addr->BSR = GPIO_addr->BSR | (1 << Pin);
	}
	else
	{
		// BRR - bit reset - odmik 0x1A
		GPIO_addr->BRR = 1 << Pin;
		// resetBit(GPIO_addr->BSR, Pin);
	}
}

// function for reading the value of an input GPIO pin
uint32_t GPIO_pin_read(GPIO_device *GPIO_addr, uint32_t Pin)
{
	// odmik 0x10
	// register IDR, 32 biten, uporablja se spodnjih 16 bitov
	return (GPIO_addr->IDR) & (1 << Pin);
}

// hardcoded delay
void delay(void)
{
	volatile int d = 500000;
	while (d--)
		;
}

/* USER CODE END 4 */

/**
 * @brief    This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
    * @brief    Reports the name of the source file and the source line number
    *                 where the assert_param error has occurred.
    * @param    file: pointer to the source file name
    * @param    line: assert_param error line source number
    * @retval None
    */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
         tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
