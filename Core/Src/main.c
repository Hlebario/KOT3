/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define a 50//50
#define b 90//90
#define c 100//100
#define PI 3.1415926536

#define BUFSIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t bufferUART = 1;
uint16_t rx_buff_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//int Set_Servo_Angle(uint8_t);////////////////////////////////////////////////////////////
//void ReciveToServo(uint8_t rx_buff[1]);
void UpdatePosition(int, float);
float getTheta(int, float, float, float);
void moveTOPS(float, float, float,
              float, float, float,
              float, float, float,
              float, float, float);
void inPlace();
void stand();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*I2C1------------------------------------------------------------------------*/
#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime>>8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}
/*I2C1------------------------------------------------------------------------*/


uint8_t rx_buff[1]={0};
/*float offset[4][3] = { { 120.0, 0, 47.449 },       //FL leg
                       { 120.0, 0, 58.851 },       //FR leg
                       { 120.0, 180, 298.149 },    //BL leg
                       { 120.0, 180, 307.388 } };  //BR leg*/

/////////////////////////////////// не полный буфер ///////////////////////////////////////
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		rx_buff_len = BUFSIZE - huart->RxXferCount;
		uint8_t res = HAL_UART_Transmit_IT(&huart1, (uint8_t*)rx_buff, rx_buff_len);
		if(res == HAL_ERROR) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_ERROR - rx_buff == NULL or rx_buff_len == 0\n", 48, 1000);
		else if(res == HAL_BUSY) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_BUSY\n", 9, 1000);
		HAL_UART_AbortReceive(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		//ReciveToServo((uint8_t*)rx_buff);////////////////////////////////////////////////////////
	}
}

/////////////////////////////////// полный буфер ///////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart1)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart1);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  //ReciveToServo((uint8_t*)rx_buff);////////////////////////////////////////////////////////
	  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// ErrorCallback //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		uint32_t er = HAL_UART_GetError(&huart1);
		HAL_UART_Abort_IT(&huart1);

		switch(er)
		{
			case HAL_UART_ERROR_PE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Parity error\n", 27, 1000);
				__HAL_UART_CLEAR_PEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_NE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Noise error\n", 26, 1000);
				__HAL_UART_CLEAR_NEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_FE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Frame error\n", 26, 1000);
				__HAL_UART_CLEAR_FEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_ORE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Overrun error\n", 28, 1000);
				__HAL_UART_CLEAR_OREFLAG(huart);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_DMA:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - DMA transfer error\n", 33, 1000);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			default:
			break;
		}
	}

}

float getTheta(int joint, float X, float Y, float Z)
{
	float L = 0;
	float angle = 0;

	L = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
	//calculates abad, hip, or knee angle
	switch (joint)
	{
    case 0:  //abad
      angle = atan(X / Z) + acos(a / (sqrt(pow(X, 2) + pow(Z, 2))));
      break;
    case 1:  //hip
      angle = (PI / 2) - asin(-Y / sqrt(L - pow(a, 2))) - acos((pow(b, 2) - pow(a, 2) - pow(c, 2) + L) / (2 * b * sqrt(L - pow(a, 2))));
      break;
    case 2:  //knee
      angle = acos((pow(a, 2) + pow(b, 2) + pow(c, 2) - L) / (2 * b * c));
      break;
	}
	return (angle * (180 / PI));
    //converts angle to degrees and returns the value
}

//Fun. for calculation angle legs and transmit at fun. UpdatePosition
void moveTOPS(float X0, float Y0, float Z0,    //FL leg
              float X1, float Y1, float Z1,    //FR leg
              float X2, float Y2, float Z2,    //BL leg
              float X3, float Y3, float Z3)	   //BR leg
{
  float relPos = 0;                                //odrive relative position
  //X, Y, Z positions of each foot
  float pos[4][3] = { { X0, Y0, Z0 },    //FL leg
                      { X1, Y1, Z1 },    //FR leg
                      { X2, Y2, Z2 },    //BL leg
                      { X3, Y3, Z3 } };  //BR leg
  //move each actuator
  for (int i = 0; i < 4; i++)
  {                                                  //cycle through each leg (FL, FR, BL, BR)
    for (int j = 0; j < 3; j++)
    {
    	//cycle through each legs joints (abad, hip, knee)
    	/*float dir[4][3] = { { 1, 1, 1 },     //FL leg
    	                    { -1, -1, -1 },  //FR leg
    	                    { -1, -1, -1 },  //BL leg
    	                    { 1, 1, 1 } };*/
    	relPos = getTheta(j, pos[i][0], pos[i][1], pos[i][2]);// - offset[i][j];  //calculate relative joint position and make positive
    	//relPos = fabs(relPos / 360.0 * GR) * dir[i][j];                            //converts relative joint position from degrees to counts using gear ratio
    	//odriveCAN.SetPosition(i * 3 + j, relPos);                                  //moves the actuator to its joint position (axisID, relative position)
    	//UpdatePosition(theta_abad* 180 / PI, theta_hip* 180 / PI, theta_knee* 180 / PI);
    	/*if (i > 1)
    	{
    		UpdatePosition(i * 3 + j, relPos);
    	}
    	else
    	{
    		UpdatePosition(i * 3 + j, 180 - relPos);
    	}*/
    	UpdatePosition(i * 3 + j, relPos);
    	HAL_UART_Receive_DMA(&huart1, rx_buff, 1);
    }
  }
}

//trots in place
void inPlace()
{
	/*int normYF = -0;//-50
	int normYB = -0;//-50//-110
	int offset = 3;//100//200
	//int offsetY = 20;//80//300
	int time = 1000;//200//160
	int normX = 50;//110
	int normZ = 150;//300*/
	int normY = 0;//-50//-110
	int offset = 20;//100//200
	//int offsetY = 20;//80//300
	int time = 1000;//200//160
	int normX = 50;//110
	int normZ = 140;//300
	moveTOPS(normX, normY, normZ - offset,   //FL
			normX, normY, normZ,                 //FR
			normX, normY, normZ - offset,                 //BL
			normX, normY, normZ);  //BR
	HAL_Delay(time);
	moveTOPS(normX, normY, normZ,                                              //FL
			normX, normY, normZ,  //FR
			normX, normY, normZ,                          //BL
			normX, normY, normZ);                                             //BR
    HAL_Delay(time);
}

void step()
{
  int normY = -0;//-50//-110
  int offset = 30;//100//200
  int time = 1;//200//160
  int normX = 50;//110
  int normZ = 140;//300

  for(int i = 0; i <= offset; i=i+3)
  {
	  moveTOPS(normX, normY, normZ - i,   //FL
	           normX, normY, normZ,                 //FR
	           normX, normY, normZ,                 //BL
	           normX, normY, normZ + i);  //BR
	  HAL_Delay(time);
  }
  for(int i = 0; i <= offset; i=i+3)
  {
	    moveTOPS(normX, normY + i, normZ - offset,                           //FL
	             normX, normY, normZ,                                                             //FR
	             normX, normY, normZ,                                                             //BL
	             normX, normY - i, normZ + offset);  //BR
	    HAL_Delay(time);
  }
  for(int i = offset; i >= 0; i=i-3)
  {
	    moveTOPS(normX, normY + offset, normZ - i,                           //FL
	             normX, normY, normZ,                                               //FR
	             normX, normY, normZ,                                               //BL
	             normX, normY - offset, normZ + i);  //BR
	    HAL_Delay(time);
  }
  for(int i = offset; i >= 0; i=i-3)
    {
  	    moveTOPS(normX, normY + i, normZ,                           //FL
  	             normX, normY, normZ,                                                             //FR
  	             normX, normY, normZ,                                                             //BL
  	             normX, normY - i, normZ);  //BR
  	    HAL_Delay(time);
    }
  for(int i = 0; i <= offset; i=i+3)
  {
	    moveTOPS(normX, normY, normZ,                //FL
	             normX, normY, normZ + i,  //FR
	             normX, normY, normZ - i,  //BL
	             normX, normY, normZ);               //BR
	    HAL_Delay(time);
  }
  for(int i = 0; i <= offset; i=i+3)
  {
	    moveTOPS(normX, normY, normZ,                                                            //FL
	             normX, normY - i, normZ + offset,  //FR
	             normX, normY + i, normZ - offset,                          //BL
	             normX, normY, normZ);                                                           //BR
	    HAL_Delay(time);
  }
  for(int i = offset; i >= 0; i=i-3)
    {
  	    moveTOPS(normX, normY, normZ,                //FL
  	             normX, normY - offset, normZ + i,  //FR
  	             normX, normY + offset, normZ - i,  //BL
  	             normX, normY, normZ);               //BR
  	    HAL_Delay(time);
    }
    for(int i = offset; i >= 0; i=i-3)
    {
  	    moveTOPS(normX, normY, normZ,                                                            //FL
  	             normX, normY - i, normZ,  //FR
  	             normX, normY + i, normZ,                          //BL
  	             normX, normY, normZ);                                                           //BR
  	    HAL_Delay(time);
    }
}

void stand()
{
	int normY = 0;
	int normX = 50;
	int normZ = 140;
	int time = 150;
	  moveTOPS(normX, normY, normZ,   //FL
	           normX, normY, normZ,                 //FR
	           normX, normY, normZ,                 //BL
	           normX, normY, normZ);  //BR
	  HAL_Delay(time);
}

//void UpdatePosition(float theta_abad, float theta_hip, float theta_knee)
void UpdatePosition(int j, float relPos)
{
//abad плечо
//hip бедро
//knee колено
	switch(j)
	{
	//Передняя левая
	case 0:
	PCA9685_SetServoAngle(0, 180 - relPos);//-20
		///TIM2 ->CCR1 = relPos + 750 - 50;//плечо
		break;
	case 1:
	PCA9685_SetServoAngle(1, 90 - relPos);//120
		//TIM2 ->CCR2 = relPos + 750 - 300;//бедро
		break;
	case 2:
	PCA9685_SetServoAngle(2, relPos);//-30
		//TIM2 ->CCR3 = relPos + 750;//колено 1300 200
		break;
	//Предняя правая
	case 3:
	PCA9685_SetServoAngle(3, 180 - relPos);
		//TIM2 ->CCR4 = relPos + 750 - 470;
		break;
	case 4:
	PCA9685_SetServoAngle(4, 90 - relPos);//20
		//TIM3 ->CCR1 = relPos + 750 - 30;
		break;
	case 5:
	PCA9685_SetServoAngle(5, relPos);//40
		//TIM3 ->CCR2 = relPos + 750 - 250;
		break;
	//Звдняя левая
	case 6:
	PCA9685_SetServoAngle(6, 180 - relPos);//-10
		//TIM3 ->CCR3 = relPos + 750 - 150;
		break;
	case 7:
	PCA9685_SetServoAngle(7, 90 - relPos);//70
		//TIM3 ->CCR4 = relPos + 750 + 250;
		break;
	case 8:
	PCA9685_SetServoAngle(8, relPos);//-50
		//TIM4 ->CCR1 = relPos + 750 - 600;
		break;
	//Задняя правая
	case 9:
	PCA9685_SetServoAngle(9, 180 - relPos);
		//TIM4 ->CCR2 = relPos + 750 - 350;
		break;
	case 10:
	PCA9685_SetServoAngle(10, 90 - relPos);//-20
		//TIM4 ->CCR3 = relPos + 750 - 450;
		break;
	case 11:
	PCA9685_SetServoAngle(11, relPos);
		//TIM5 ->CCR1 = relPos + 750 + 200;
		//TIM4 ->CCR1 = relPos + 750;
		break;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*I2C1------------------------------------------------------------------------*/
  PCA9685_Init(50); // 50Hz for servo
  /*I2C1------------------------------------------------------------------------*/

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, rx_buff, bufferUART);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  step();
	  //stand();
	  //inPlace();*/
	  /*GPIOA -> ODR |= (1 << 6);
	  HAL_Delay(500);
	  GPIOA -> ODR |= (1 << 7);
	  HAL_Delay(500);
	  GPIOA -> ODR &= ~(1 << 6);
	  HAL_Delay(500);
	  GPIOA -> ODR &= ~(1 << 7);
	  HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
