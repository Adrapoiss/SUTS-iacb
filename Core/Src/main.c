
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lis3mdl.h"
#include "custom_motion_sensors.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

/* Define the LIS3MDL ID */
#define I_AM_LIS3MDL 0x1C // Define the expected ID for the LIS3MDL

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; //uart com1 koduarvutil.
int16_t magnetometer_data[3]; // 3 dim andmete arr
LIS3MDL_Object_t lis3mdl; //object struct pm

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Read_Magnetometer(void);
void Init_Magnetometer(void);

/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

//wrapper funktsioonid lugemiseks ja kirjutamiseks et saada mingit tagasisidet
int32_t My_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
    printf("Writing to I2C: Addr=0x%02X, Reg=0x%02X, Length=%d\r\n", Addr, Reg, Length);
    int32_t result = HAL_I2C_Mem_Write(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT);

    if (result != HAL_OK) {
        printf("I2C Write Error: %ld\r\n", result);
    } else {
        printf("I2C Write Successful\r\n");
    }

    return result;
}

int32_t My_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
    printf("Reading from I2C: Addr=0x%02X, Reg=0x%02X, Length=%d\r\n", Addr, Reg, Length);
    int32_t result = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT);

    if (result != HAL_OK) {
        printf("I2C Read Error: %ld\r\n", result);
    } else {
        printf("I2C Read Successful: Data=0x%02X\r\n", *pData); //esimene bait tuleb väljundina
    }

    return result;
}
void Test_I2C_Communication(void) {
    uint8_t data;
    HAL_StatusTypeDef status;
    printf("Testing I2C connection... \r\n");

    // 0x0F ehk who am i registeri lugemine, kõige lihtsam test I2C ühenduse kontrollimiseks.
    status = HAL_I2C_Mem_Read(&hi2c1, 0x1C, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 100); //Viimane nr on päringu timeout

    if (status == HAL_OK) {
        printf("I2C Read Successful: WHO_AM_I = 0x%02X\r\n", data);
    } else {
        // Check for specific HAL error codes
        if (status == HAL_TIMEOUT) {
            printf("I2C Read Timeout occurred in Test_I2C_Communication\r\n");
        } else {
            printf("I2C Read Error in Test_I2C_Communication: %d\r\n", status); //Hetkeseisuga saan siit HAL_ERROR ehk status = 1
        }
    }
}
void Init_Magnetometer(void) {
    printf("Initializing Magnetometer...\r\n");
    LIS3MDL_IO_t io;
    io.BusType = 0; // 0 on i2c 1 ja 2 on SPI
    io.Address = 0x1C;

    //Esimesed kaks wrapperis
    io.WriteReg = My_WriteReg;
    io.ReadReg = My_ReadReg;
    io.GetTick = (LIS3MDL_GetTick_Func)HAL_GetTick;
    io.Delay = HAL_Delay;

    //Test enne initit
    Test_I2C_Communication();

    // Register the bus I/O
    printf("Registering IO\r\n");
    if (LIS3MDL_RegisterBusIO(&lis3mdl, &io) != LIS3MDL_OK) {
        printf("Failed to register bus I/O\r\n");
        while (1); //Jääb loopi et mitte vigaste aadressidega edasi toimetada
    }

    // Initialize the LIS3MDL
    printf("Initializing LIS3MDL...\r\n");
    if (LIS3MDL_Init(&lis3mdl) != LIS3MDL_OK) {
        printf("Magnetometer initialization failed\r\n");
        while (1);
    }

    // Check magnetometer ID
    uint8_t id;
    if (LIS3MDL_ReadID(&lis3mdl, &id) != LIS3MDL_OK) {
        printf("Failed to read magnetometer ID\r\n");
        while (1);
    }

    printf("Magnetometer detected with ID: %d\r\n", id);
}

void Read_Magnetometer(void) {

    if (LIS3MDL_MAG_GetAxes(&lis3mdl, (LIS3MDL_Axes_t *)magnetometer_data) != LIS3MDL_OK) {
        printf("Failed to read magnetometer data\r\n");
        return; // kui ei tule vastust läheb tagasi maini
    }

    // Print magnetometer data for debugging
    printf("X: %d, Y: %d, Z: %d\r\n", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init(&hi2c1); //

    printf("----------Starting------------\r\n");


    //Magnetomeetri init
    Init_Magnetometer();

    /* Infinite loop */
    while (1)
    {
        Read_Magnetometer(); //proovib lugeda ja 1 s delay
        HAL_Delay(1000); //ms
    }
}

HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef* hi2c) {
    hi2c->Instance = I2C1;
    hi2c->Init.Timing = 0x2000090E; //200Khz
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_StatusTypeDef result = HAL_I2C_Init(hi2c); //HAL I2C init
    if (result != HAL_OK) {
        printf("I2C Initialization Error: %d\r\n", result);
    }
    else{
    	printf("I2C Initialization done\r\n");
    }
    return result; //siit saab läbi niiet MCU jaoks peaks I2C kenasti seadistatud olema
}
//Järgnev on STMCubeIDE poolt genereeritud pordi init, osc init jms.
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct );
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
