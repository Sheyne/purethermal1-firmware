/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include <stdint.h>
#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

#include "tasks.h"
#include "project_config.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif
  
static struct pt lepton_task_pt;
static struct pt usb_task_pt;
static struct pt convert_task_pt;
static struct pt uart_task_pt;
static struct pt button_task_pt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  // reinitialize uart with speed from config
  huart2.Init.BaudRate = USART_DEBUG_SPEED;
  HAL_UART_Init(&huart2);

  DEBUG_PRINTF("Hello, Lepton!\n\r");
  fflush(stdout);

  lepton_init();

  HAL_Delay(1000);

  init_lepton_command_interface();
#ifdef ENABLE_LEPTON_AGC
  enable_lepton_agc();
#endif

	LEP_VID_LUT_BUFFER_T user_lut = { .bin = { { 0, 0, 0, 0 }, { 1, 1, 1, 1 }, {
			2, 2, 2, 2 }, { 3, 3, 3, 3 }, { 4, 4, 4, 4 }, { 5, 5, 5, 5 }, { 6,
			6, 6, 6 }, { 7, 7, 7, 7 }, { 8, 8, 8, 8 }, { 9, 9, 9, 9 }, { 10, 10,
			10, 10 }, { 11, 11, 11, 11 }, { 12, 12, 12, 12 },
			{ 13, 13, 13, 13 }, { 14, 14, 14, 14 }, { 15, 15, 15, 15 }, { 16,
					16, 16, 16 }, { 17, 17, 17, 17 }, { 18, 18, 18, 18 }, { 19,
					19, 19, 19 }, { 20, 20, 20, 20 }, { 21, 21, 21, 21 }, { 22,
					22, 22, 22 }, { 23, 23, 23, 23 }, { 24, 24, 24, 24 }, { 25,
					25, 25, 25 }, { 26, 26, 26, 26 }, { 27, 27, 27, 27 }, { 28,
					28, 28, 28 }, { 29, 29, 29, 29 }, { 30, 30, 30, 30 }, { 31,
					31, 31, 31 }, { 32, 32, 32, 32 }, { 33, 33, 33, 33 }, { 34,
					34, 34, 34 }, { 35, 35, 35, 35 }, { 36, 36, 36, 36 }, { 37,
					37, 37, 37 }, { 38, 38, 38, 38 }, { 39, 39, 39, 39 }, { 40,
					40, 40, 40 }, { 41, 41, 41, 41 }, { 42, 42, 42, 42 }, { 43,
					43, 43, 43 }, { 44, 44, 44, 44 }, { 45, 45, 45, 45 }, { 46,
					46, 46, 46 }, { 47, 47, 47, 47 }, { 48, 48, 48, 48 }, { 49,
					49, 49, 49 }, { 50, 50, 50, 50 }, { 51, 51, 51, 51 }, { 52,
					52, 52, 52 }, { 53, 53, 53, 53 }, { 54, 54, 54, 54 }, { 55,
					55, 55, 55 }, { 56, 56, 56, 56 }, { 57, 57, 57, 57 }, { 58,
					58, 58, 58 }, { 59, 59, 59, 59 }, { 60, 60, 60, 60 }, { 61,
					61, 61, 61 }, { 62, 62, 62, 62 }, { 63, 63, 63, 63 }, { 64,
					64, 64, 64 }, { 65, 65, 65, 65 }, { 66, 66, 66, 66 }, { 67,
					67, 67, 67 }, { 68, 68, 68, 68 }, { 69, 69, 69, 69 }, { 70,
					70, 70, 70 }, { 71, 71, 71, 71 }, { 72, 72, 72, 72 }, { 73,
					73, 73, 73 }, { 74, 74, 74, 74 }, { 75, 75, 75, 75 }, { 76,
					76, 76, 76 }, { 77, 77, 77, 77 }, { 78, 78, 78, 78 }, { 79,
					79, 79, 79 }, { 80, 80, 80, 80 }, { 81, 81, 81, 81 }, { 82,
					82, 82, 82 }, { 83, 83, 83, 83 }, { 84, 84, 84, 84 }, { 85,
					85, 85, 85 }, { 86, 86, 86, 86 }, { 87, 87, 87, 87 }, { 88,
					88, 88, 88 }, { 89, 89, 89, 89 }, { 90, 90, 90, 90 }, { 91,
					91, 91, 91 }, { 92, 92, 92, 92 }, { 93, 93, 93, 93 }, { 94,
					94, 94, 94 }, { 95, 95, 95, 95 }, { 96, 96, 96, 96 }, { 97,
					97, 97, 97 }, { 98, 98, 98, 98 }, { 99, 99, 99, 99 }, { 100,
					100, 100, 100 }, { 101, 101, 101, 101 }, { 102, 102, 102,
					102 }, { 103, 103, 103, 103 }, { 104, 104, 104, 104 }, {
					105, 105, 105, 105 }, { 106, 106, 106, 106 }, { 107, 107,
					107, 107 }, { 108, 108, 108, 108 }, { 109, 109, 109, 109 },
			{ 110, 110, 110, 110 }, { 111, 111, 111, 111 },
			{ 112, 112, 112, 112 }, { 113, 113, 113, 113 },
			{ 114, 114, 114, 114 }, { 115, 115, 115, 115 },
			{ 116, 116, 116, 116 }, { 117, 117, 117, 117 },
			{ 118, 118, 118, 118 }, { 119, 119, 119, 119 },
			{ 120, 120, 120, 120 }, { 121, 121, 121, 121 },
			{ 122, 122, 122, 122 }, { 123, 123, 123, 123 },
			{ 124, 124, 124, 124 }, { 125, 125, 125, 125 },
			{ 126, 126, 126, 126 }, { 127, 127, 127, 127 },
			{ 128, 128, 128, 128 }, { 129, 129, 129, 129 },
			{ 130, 130, 130, 130 }, { 131, 131, 131, 131 },
			{ 132, 132, 132, 132 }, { 133, 133, 133, 133 },
			{ 134, 134, 134, 134 }, { 135, 135, 135, 135 },
			{ 136, 136, 136, 136 }, { 137, 137, 137, 137 },
			{ 138, 138, 138, 138 }, { 139, 139, 139, 139 },
			{ 140, 140, 140, 140 }, { 141, 141, 141, 141 },
			{ 142, 142, 142, 142 }, { 143, 143, 143, 143 },
			{ 144, 144, 144, 144 }, { 145, 145, 145, 145 },
			{ 146, 146, 146, 146 }, { 147, 147, 147, 147 },
			{ 148, 148, 148, 148 }, { 149, 149, 149, 149 },
			{ 150, 150, 150, 150 }, { 151, 151, 151, 151 },
			{ 152, 152, 152, 152 }, { 153, 153, 153, 153 },
			{ 154, 154, 154, 154 }, { 155, 155, 155, 155 },
			{ 156, 156, 156, 156 }, { 157, 157, 157, 157 },
			{ 158, 158, 158, 158 }, { 159, 159, 159, 159 },
			{ 160, 160, 160, 160 }, { 161, 161, 161, 161 },
			{ 162, 162, 162, 162 }, { 163, 163, 163, 163 },
			{ 164, 164, 164, 164 }, { 165, 165, 165, 165 },
			{ 166, 166, 166, 166 }, { 167, 167, 167, 167 },
			{ 168, 168, 168, 168 }, { 169, 169, 169, 169 },
			{ 170, 170, 170, 170 }, { 171, 171, 171, 171 },
			{ 172, 172, 172, 172 }, { 173, 173, 173, 173 },
			{ 174, 174, 174, 174 }, { 175, 175, 175, 175 },
			{ 176, 176, 176, 176 }, { 177, 177, 177, 177 },
			{ 178, 178, 178, 178 }, { 179, 179, 179, 179 },
			{ 180, 180, 180, 180 }, { 181, 181, 181, 181 },
			{ 182, 182, 182, 182 }, { 183, 183, 183, 183 },
			{ 184, 184, 184, 184 }, { 185, 185, 185, 185 },
			{ 186, 186, 186, 186 }, { 187, 187, 187, 187 },
			{ 188, 188, 188, 188 }, { 189, 189, 189, 189 },
			{ 190, 190, 190, 190 }, { 191, 191, 191, 191 },
			{ 192, 192, 192, 192 }, { 193, 193, 193, 193 },
			{ 194, 194, 194, 194 }, { 195, 195, 195, 195 },
			{ 196, 196, 196, 196 }, { 197, 197, 197, 197 },
			{ 198, 198, 198, 198 }, { 199, 199, 199, 199 },
			{ 200, 200, 200, 200 }, { 201, 201, 201, 201 },
			{ 202, 202, 202, 202 }, { 203, 203, 203, 203 },
			{ 204, 204, 204, 204 }, { 205, 205, 205, 205 },
			{ 206, 206, 206, 206 }, { 207, 207, 207, 207 },
			{ 208, 208, 208, 208 }, { 209, 209, 209, 209 },
			{ 210, 210, 210, 210 }, { 211, 211, 211, 211 },
			{ 212, 212, 212, 212 }, { 213, 213, 213, 213 },
			{ 214, 214, 214, 214 }, { 215, 215, 215, 215 },
			{ 216, 216, 216, 216 }, { 217, 217, 217, 217 },
			{ 218, 218, 218, 218 }, { 219, 219, 219, 219 },
			{ 220, 220, 220, 220 }, { 221, 221, 221, 221 },
			{ 222, 222, 222, 222 }, { 223, 223, 223, 223 },
			{ 224, 224, 224, 224 }, { 225, 225, 225, 225 },
			{ 226, 226, 226, 226 }, { 227, 227, 227, 227 },
			{ 228, 228, 228, 228 }, { 229, 229, 229, 229 },
			{ 230, 230, 230, 230 }, { 231, 231, 231, 231 },
			{ 232, 232, 232, 232 }, { 233, 233, 233, 233 },
			{ 234, 234, 234, 234 }, { 235, 235, 235, 235 },
			{ 236, 236, 236, 236 }, { 237, 237, 237, 237 },
			{ 238, 238, 238, 238 }, { 239, 239, 239, 239 },
			{ 240, 240, 240, 240 }, { 241, 241, 241, 241 },
			{ 242, 242, 242, 242 }, { 243, 243, 243, 243 },
			{ 244, 244, 244, 244 }, { 245, 245, 245, 245 },
			{ 246, 246, 246, 246 }, { 247, 247, 247, 247 },
			{ 248, 248, 248, 248 }, { 249, 249, 249, 249 },
			{ 250, 250, 250, 250 }, { 251, 251, 251, 251 },
			{ 252, 252, 252, 252 }, { 253, 253, 253, 253 },
			{ 254, 254, 254, 254 }, { 255, 255, 255, 255 } } };

  set_user_lut(&user_lut);
  enable_rgb888(LEP_VID_USER_LUT);


  DEBUG_PRINTF("reading_tmp007_regs...\n\r");

  read_tmp007_regs();

  DEBUG_PRINTF("Initialized...\n\r");

  HAL_Delay(250);

  MX_USB_DEVICE_Init();

  PT_INIT(&lepton_task_pt);
  PT_INIT(&usb_task_pt);
  PT_INIT(&uart_task_pt);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  PT_SCHEDULE(lepton_task(&lepton_task_pt));
	  PT_SCHEDULE(convert_task(&convert_task_pt));
	  PT_SCHEDULE(usb_task(&usb_task_pt));
	  PT_SCHEDULE(uart_task(&uart_task_pt));
	  PT_SCHEDULE(button_task(&button_task_pt));

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 11;
  RCC_OscInitStruct.PLL.PLLN = 295;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  HAL_CRC_Init(&hcrc);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&hdma_memtomem_dma2_stream0);

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : LEPTON_GPIO3_Pin */
  GPIO_InitStruct.Pin = LEPTON_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEPTON_GPIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_GPIO2_Pin ESP_GPIO0_Pin ESP_CH_PD_Pin SYSTEM_LED_Pin 
                           ESP_RST_Pin LEPTON_PW_DWN_L_Pin LEPTON_RESET_L_Pin */
  GPIO_InitStruct.Pin = ESP_GPIO2_Pin|ESP_GPIO0_Pin|ESP_CH_PD_Pin|SYSTEM_LED_Pin 
                          |ESP_RST_Pin|LEPTON_PW_DWN_L_Pin|LEPTON_RESET_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_STAT_Pin */
  GPIO_InitStruct.Pin = PB_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUCK_ON_Pin LDO_ON_Pin */
  GPIO_InitStruct.Pin = BUCK_ON_Pin|LDO_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ESP_GPIO2_Pin|ESP_GPIO0_Pin|ESP_CH_PD_Pin|SYSTEM_LED_Pin 
                          |ESP_RST_Pin|LEPTON_PW_DWN_L_Pin|LEPTON_RESET_L_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
