/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_UART1 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const int16_t rx_buffer_size = 512;

uint8_t uart2_rx_buffer[rx_buffer_size] = {'\0'};
uint8_t uart2_rx_byte = '\0';
int16_t uart2_rx_index = 0;

uint8_t uart1_rx_buffer[rx_buffer_size] = {'\0'};
uint8_t uart1_rx_byte = '\0';
int16_t uart1_rx_index = 0;

int16_t current_i = 1;
char url[40] = "http://1.1.1.1:8080/sms";

char sms_oa[30];
char sms_scts[30];
char sms_data[281];
char sms_da[15] = "13512345678";

int16_t new_sms = 0;
int16_t run_cnt = 0;
int16_t gprs_stat = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if (huart->Instance == USART1) {
    if (uart1_rx_index > (rx_buffer_size - 1)) {
      uart1_rx_index = 0;
    }
    uart1_rx_buffer[uart1_rx_index++] = uart1_rx_byte;
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
  if (huart->Instance == USART2) {
    if (uart2_rx_index > (rx_buffer_size - 1)) {
      uart2_rx_index = 0;
    }
    uart2_rx_buffer[uart2_rx_index++] = uart2_rx_byte;
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  }

}

int8_t SendAtCmd(const char *cmd, const char *ack, int16_t timeout) {
 
  uart2_rx_index = 0;
  uart2_rx_buffer[0] = '\0';
  int8_t ret = 0;

  if(HAL_OK != HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 100)){
		return ret;
	}

  while (timeout > 0 && ret == 0) {
    if (strstr((char *)uart2_rx_buffer, ack)) {
      ret = 1;
    }
    HAL_Delay(100);
		timeout -= 100;
  }
  return ret;
}

void Debug(const char *info) {
  char CRLF[3] = "\r\n";
  HAL_Delay(100);
  HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)CRLF, strlen(CRLF), 100);
  HAL_Delay(100);
}

int8_t GsmInit(void) {
  int8_t ok = 1;

  if (ok == 1) {
    ok = SendAtCmd("AT\r\n", "OK",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT+CPIN?\r\n", "READY",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT+CSQ\r\n", "+CSQ",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT+CREG?\r\n", "+CREG",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT+CMGF=1\r\n", "OK",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT&W\r\n", "OK",1000);
  }

  if (ok == 1) {
    ok = SendAtCmd("AT+SAPBR=0,1\r\n", "OK",1000);
  }

  return ok;
}

int8_t GprsInit(void) {
  int8_t ok = 1;

  if (ok == 1) {
    ok = SendAtCmd("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK",1000);
  }
	HAL_Delay(100);
	
  if (ok == 1) {
    ok = SendAtCmd("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n", "OK",1000);
  }
  HAL_Delay(2000);
	
  if (ok == 1) {
    ok = SendAtCmd("AT+SAPBR=1,1\r\n", "OK",1000);
  }

  if (ok == 1) {
    gprs_stat = 1;
  }

  HAL_Delay(2000);
  return ok;
}

int8_t GprsEnd(void) {
  int8_t ok = 1;

  ok = SendAtCmd("AT+SAPBR=0,1\r\n", "OK",1000);

  if (ok == 1) {
    gprs_stat = 0;
  }

  return ok;
}

int8_t HttpInit(void) {
  int8_t ok = 1;

  if (ok == 1) {
    ok = SendAtCmd("AT+HTTPINIT\r\n", "OK",1000);
  }
	HAL_Delay(100);
	
  if (ok == 1) {
    ok = SendAtCmd("AT+HTTPPARA=\"CID\",1\r\n", "OK",1000);
  }
	HAL_Delay(100);

  if (ok == 1) {
    ok = SendAtCmd(
        "AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"\r\n",
        "OK",1000);
  }
  return ok;
}

int8_t HttpSend() {
  int8_t ok = 1;

  char set_web_server_url_cmd[80];
  char set_data_length_cmd[30];
  char data_cmd[512];

  sprintf(set_web_server_url_cmd, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
  sprintf(data_cmd, "oa=%s&scts=%s&data=%s&da=%s", sms_oa, sms_scts, sms_data,
          sms_da);
  sprintf(set_data_length_cmd, "AT+HTTPDATA=%d,10000\r\n", strlen(data_cmd));

  if (ok == 1) {
    ok = SendAtCmd(set_web_server_url_cmd, "OK",1000);
  }
	HAL_Delay(1000);

  if (ok == 1) {
    ok = SendAtCmd(set_data_length_cmd, "DOWNLOAD",1000);
  }
	HAL_Delay(1000);

  if (ok == 1) {
    ok = SendAtCmd(data_cmd, "OK",1000);
  }
	HAL_Delay(1000);

  if (ok == 1) {
    ok = SendAtCmd("AT+HTTPACTION=1\r\n", "+HTTPACTION: 1,200",5000);
  }
  return ok;
}

int8_t HttpEnd(void) {
  return SendAtCmd("AT+HTTPTERM\r\n", "OK",2000);
}

void OaUrlDecode(void) {
  char c, oa_[30];
  sscanf((char *)sms_oa, "%c%s", &c, oa_);
  if (c == '+') {
    strcpy(sms_oa, oa_);
  }
}

void SctsUrlDecode(void) {
  int intzone, max = 6;
  char timestamp[max][3], c, result[50];
  char t[50];

  sscanf((char *)sms_scts, "%2s/%2s/%2s,%2s:%2s:%2s%c%d", timestamp[0],
         timestamp[1], timestamp[2], timestamp[3], timestamp[4], timestamp[5],
         &c, &intzone);

  intzone = intzone / 4;
  memset(result, '\0', sizeof(result));

  for (int i = 0; i < max; i++) {
    strcat(result, timestamp[i]);
  }

  if (c == '+') {
    strcat(result, "%2B");
  } else if (c == '-') {
    strcat(result, "-");
  }
  if (intzone > 9) {
    // strcat(result, strzone);
    sprintf(t, "%s%d", result, intzone);
  } else {
    // strcat(result, "0");
    // strcat(result, strzone);
    sprintf(t, "%s0%d", result, intzone);
  }
  memset(sms_scts, '\0', sizeof(sms_scts));
  strcpy(sms_scts, t);
}

int8_t ReadSms(void) {
  char *p;
  int sms_used1, sms_total1, sms_used2, sms_total2, sms_used3, sms_total3, res;
  char oa[30], scts[30], data[281];
  char read_sms_cmd[15];
  char debug_info[50];
  char data_[512];
  char data1_[201];
  char data2_[201];
  char CTRLZ[] = {26};

  SendAtCmd("AT+CPMS?\r\n", "+CPMS:",1000);
  p = strstr((char *)uart2_rx_buffer, "+CPMS:");
  if (p) {
    sscanf(p, "%*[^,],%d,%d,%*[^,],%d,%d,%*[^,],%d,%d", &sms_used1, &sms_total1,
           &sms_used2, &sms_total2, &sms_used3, &sms_total3);

    sprintf(debug_info,"sms:(%d,%d);(%d,%d);(%d,%d).",sms_used1, sms_total1, sms_used2, sms_total2, sms_used3, sms_total3);
  }

  if (sms_used1 > 0) {
    for (int i = 1; i < sms_total1; i++) {
      sprintf(read_sms_cmd, "AT+CMGR=%d\r\n", i);
      if (SendAtCmd(read_sms_cmd, "+CMGR",1000)) {
        current_i = i;
        p = strstr((char *)uart2_rx_buffer, "+CMGR");
        if (p) {
          res = sscanf(p, "%*[^,],\"%[^\"]\",%*[^,],\"%[^\"]\"\r\n%[^\r]", oa,
                       scts, data);
          if (res == 3) {
            strcpy(sms_oa, oa);
            strcpy(sms_scts, scts);
            strcpy(sms_data, data);

            OaUrlDecode();
            SctsUrlDecode();

            #if !DEBUG_UART1
            // send to uart1 esp8266
            sprintf(data_, "oa=%s&scts=%s&data=%s&da=%s", sms_oa, sms_scts, sms_data,sms_da);

            if(strlen(data_) > 200){
              sscanf((char*)data_,"%200s%s",data1_,data2_);
              Debug(data1_);
              HAL_Delay(100);
              Debug(data2_);
            }else{
              Debug(data_);
            }
            HAL_Delay(100);

            HAL_UART_Transmit(&huart1, (uint8_t *)CTRLZ, strlen(CTRLZ), 100);
            
            // send end.
            #endif
						
            return 1;
          }
        }
      }
    }
  }
  return 0;
}

int8_t DeleteSms(void) {
  char delete_sms_cmd[15];

  sprintf(delete_sms_cmd, "AT+CMGD=%d\r\n", current_i);

  return SendAtCmd(delete_sms_cmd, "OK",1000);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);

  HAL_Delay(5000);  // startup module need delay

  GsmInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(strstr((char*)uart2_rx_buffer,"+CMTI:")){
			new_sms = 1;
	
		}else if(run_cnt % 20 == 0){
				if(ReadSms()){
				new_sms = 1;
			}
		}
		
		if(new_sms)
		{
				
					if(GprsInit())
					{
						
								if(HttpInit())
								{
									
											while(ReadSms())
											{
												
													HttpSend();
														
													HAL_Delay(200);
														
													DeleteSms();
												
											}
											
											HttpEnd();
								}
								
								GprsEnd();
					}
					
					new_sms = 0;
		}
			
		if(run_cnt > 30000){
			run_cnt =0;
		}
		
		run_cnt++;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(64);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(3000);
		
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
  /* User can add his own implementation to report the file name and line
   number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
   line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
