/* USER CODE BEGIN Header */
// ... (Header remains the same)
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lis3mdl.h" 
#include "stdio.h"  
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIS3MDL_I2C_ADDRESS LIS3MDL_I2C_ADD_L // Or LIS3MDL_I2C_ADD_L if SA0 is low
#define BUS_I2C1_POLL_TIMEOUT 100 // Define the timeout
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

UART_HandleTypeDef huart2;
LIS3MDL_Object_t lis3mdl; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LIS3MDL_CS_Select(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 
}

void LIS3MDL_CS_Deselect(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

int32_t Write_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    reg |= 0x40; // Multi-write bit
    LIS3MDL_CS_Select();
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, data, len, HAL_MAX_DELAY);
    LIS3MDL_CS_Deselect();
    return LIS3MDL_OK;
}

int32_t Read_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    reg |= 0x80 | 0x40; // Multi-read bit
    LIS3MDL_CS_Select();
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, data, len, HAL_MAX_DELAY);
    LIS3MDL_CS_Deselect();
    return LIS3MDL_OK;
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void Read_Magnetometer(void) {
    LIS3MDL_Axes_t axes;
    if (LIS3MDL_MAG_GetAxes(&lis3mdl, &axes) == LIS3MDL_OK) {
        printf("X: %ld, Y: %ld, Z: %ld\r\n", axes.x, axes.y, axes.z); // Correct format specifier
    } else {
        printf("Failed to read magnetometer data\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  printf("----------Starting SPI------------\r\n");

      LIS3MDL_IO_t io; //I/O init
      io.BusType = LIS3MDL_SPI_4WIRES_BUS; //sõltuvalt kas mosi on kasutusel
      io.Address = 0; //spi seega aadress puudub aga et ei jääks tühjaks
      io.WriteReg = (LIS3MDL_WriteReg_Func)Write_LIS3MDL;
      io.ReadReg = (LIS3MDL_ReadReg_Func)Read_LIS3MDL;
      io.GetTick = (LIS3MDL_GetTick_Func)(uint32_t (*)())HAL_GetTick;; //Terve aeg oli vist siin viga sees
      io.Delay = HAL_Delay;
      io.Init = NULL; //Hal seega pole vaja
      io.DeInit = NULL;

      lis3mdl.Ctx.handle = &hspi2;
      lis3mdl.Ctx.write_reg = Write_LIS3MDL;
      lis3mdl.Ctx.read_reg = Read_LIS3MDL;
      lis3mdl.IO = io; // need neli olid ka puudu

      uint8_t id;
      if (LIS3MDL_ReadID(&lis3mdl, &id) == LIS3MDL_OK) {
          printf("LIS3MDL WHO_AM_I: 0x%02X\r\n", id);
          if (id != LIS3MDL_ID) {
              printf("Incorrect WHO_AM_I value!\r\n");
              Error_Handler();
          }
      } else {
          printf("Could not read WHO_AM_I\r\n");
          Error_Handler();
      }

      if (LIS3MDL_Init(&lis3mdl) != LIS3MDL_OK) { //Init
          printf("LIS3MDL_Init failed\r\n");
          Error_Handler();
      }
      if (lis3mdl_operating_mode_set(&lis3mdl.Ctx, LIS3MDL_CONTINUOUS_MODE) != LIS3MDL_OK) { //Koguaeg mõõdab
          printf("Error setting operating mode\r\n");
          Error_Handler();
      }
      if (lis3mdl_data_rate_set(&lis3mdl.Ctx, LIS3MDL_UHP_155Hz) != LIS3MDL_OK) { //Kõige suurem
          printf("Error setting data rate\r\n");
          Error_Handler();
      }
      if (LIS3MDL_MAG_SetFullScale(&lis3mdl, 4) != LIS3MDL_OK) { //4 gauss täpsus
          printf("Error setting full scale\r\n");
          Error_Handler();
      }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      LIS3MDL_Axes_t mag;
          lis3mdl_axis3bit16_t mag_raw; // Correct type for raw data
          float sensitivity;
          float odr;

          LIS3MDL_MAG_GetOutputDataRate(&lis3mdl, &odr); // Get current ODR

          while (1) {
              if (lis3mdl_magnetic_raw_get(&lis3mdl.Ctx, mag_raw.i16bit) == LIS3MDL_OK) { // Get raw data (Correct way!)
                  // Access data using mag_raw.i16bit
                  mag.x = (int16_t)mag_raw.i16bit[0];
                  mag.y = (int16_t)mag_raw.i16bit[1];
                  mag.z = (int16_t)mag_raw.i16bit[2];

                  LIS3MDL_MAG_GetSensitivity(&lis3mdl, &sensitivity); // Get sensitivity
                  mag.x = (int32_t)((float)mag.x * sensitivity); // Scale
                  mag.y = (int32_t)((float)mag.y * sensitivity);
                  mag.z = (int32_t)((float)mag.z * sensitivity);

                  //Saab lugeda ka lihtsalt Read_Magnetometer() funktsiooniga, ise kahtlustan et loeb praegu ainult 8 bitti kuigi väljund tegelikult on vist
                  //okei (X:+-XXX Y:+-YYY Z:+-ZZZ). Seega lasin AI mingi asja siia genereerida millest väljund otseselt ei muutunud. Pean testima mõlemat.

                  printf("X: %ld, Y: %ld, Z: %ld\r\n", mag.x, mag.y, mag.z);
              } else {
                  printf("Error reading magnetometer data\r\n");
              }

              HAL_Delay((uint32_t)(1000.0f / odr)); // Delay based on ODR
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
