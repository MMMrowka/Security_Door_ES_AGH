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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX 65535
#define MIN 0
#define STABLE_ITERATIONS 40

#define RESPONSE_BYTES 2
#define ADC_RESOLUTION 4096

#define KEYPAD_READ_ADDR 0x41
#define KEYPAD_WRITE_ADDR 0x40
#define KEYPAD_GPIOA_ADDR 0x12


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
extern uint8_t overflow_flag_tim17;
extern uint8_t overflow_flag_tim7;

float power_voltage = 3.3;
uint16_t ADC_buffer[4]={0xFFFF,0xFFFF,0xFFFF,0xFFFF};
uint16_t ADC_value[2]={0,0};
uint16_t distance[2]={0,0};
uint16_t DAC_value[2]={0,0};
int control_value[2]={0,0};
uint16_t motor_refference_prim[2]={1000,1000};//0-open, 1-closed
uint16_t motor_refference[2]={1000,1000};//0-open, 1000-closed
uint8_t curtain_lock[2]={1,1};//0-open, 1-closed
uint8_t curtain_motor[2]={1,1};//0-open, 1-closed
int error[2]={0,0};
int error_prim[5]={0,0};
float kP=1;
float kD=0.5;

uint8_t recieved[3]={0,0,0};
uint16_t com_password=0xFFFF;

uint32_t dac_write_command = 0x3;
uint32_t channel_write_ch_A = 0x0;
uint32_t channel_write_ch_B = 0x1;

uint16_t standby_password=1234;
uint16_t alarm_password=2345;
uint16_t night_password=3456;
uint8_t com_locked=1;
uint8_t door_status=1; //0-opened, 1- closed
uint8_t curtain_status[2]={2,2}; //0-opened, 1-closing, 2 - closed, 3 - opening
uint8_t controller_status[2]={0,0};
uint8_t door_lock=1;//0-open, 1-closed
uint8_t status=0;//0-standby, 1-alarm, 2-nightmode
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
void open_curtains(){
    if(curtain_status[0]!=0){
    	HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_RESET);
        motor_refference[0]=0;
        curtain_status[0]=3;
        //printf("\n regulator for curtain 1 started");
        controller_status[0]=1;
        controller_status[1]=1;
        HAL_TIM_Base_Start_IT(&htim7);
    }
    if(curtain_status[1]!=0){
    	HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_RESET);
        motor_refference[1]=0;
        curtain_status[1]=3;
        //printf("\n regulator for curtain 2 started");
        controller_status[0]=1;
        controller_status[1]=1;
        HAL_TIM_Base_Start_IT(&htim7);
    }
}
void open_door(){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	door_status=0;
	HAL_TIM_Base_Start_IT(&htim17);
};
void close_all(){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
    if(curtain_status[0]!=2){
    	HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_RESET);
        motor_refference[0]=1000;
        curtain_status[0]=1;
        //printf("\n regulator for curtain 1 started");
        controller_status[0]=1;
		controller_status[1]=1;
        HAL_TIM_Base_Start_IT(&htim7);
    }
    if(curtain_status[1]!=2){
		HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_RESET);
		motor_refference[1]=1000;
		curtain_status[1]=1;
		//printf("\n regulator for curtain 1 started");
		controller_status[0]=1;
		controller_status[1]=1;
		HAL_TIM_Base_Start_IT(&htim7);
	}
}
void turn_standby(){
	status=0;
	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET);
    open_curtains();
    //printf("\nSTANDBY ON\n");
}
void turn_alarm(){
	status=1;
	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
    close_all();
    //printf("\nALARM ON\n");
}
void turn_nightmode(){
	status=2;
    close_all();
    //printf("\nNIGHTMODE ON\n");
}
uint8_t* buffer_fitting(uint32_t response){
	static uint8_t buffer[RESPONSE_BYTES];
	for(int i=0;i<RESPONSE_BYTES;i++){
		buffer[i]=(response>>((RESPONSE_BYTES-1-i)*8))&0xFF;
	}
	return buffer;
}
int COM_USART_recieve(){
    uint8_t msg_type;
    uint8_t msg_target;
    uint16_t msg_argument;
    static uint16_t response;
    uint8_t* send_buffer;
    msg_type=recieved[0]>>4;
    msg_target=recieved[0]&0x0F;
    msg_argument=recieved[1]<<8;
    msg_argument=msg_argument|recieved[2];
    /*printBits(sizeof(msg_type),&msg_type);
    printBits(sizeof(msg_target),&msg_target);
    printBits(sizeof(msg_argument),&msg_argument);*/
    if(msg_type==0&&msg_target==0&&com_locked==1){
        if(msg_argument==com_password)
        {
            com_locked=0;
            response=0;
            send_buffer=buffer_fitting(response);
            HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("access granted:");
            //printBits(sizeof(response),&response);
            return 0;
        }else{
            com_locked=1;
            response=1;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("locked:");
            //printBits(sizeof(response),&response);
            return 0;
        }
    }
    if(com_locked==1){
        response=0xF1;
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        //printf("access denied:");
        //printBits(sizeof(response),&response);
        return 1;
    }
    switch(msg_type){
    case 0://close com
    	com_locked=1;
        if(msg_target!=1){
            response=0x10;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("wrong target:");
            //printBits(sizeof(response),&response);
            return 1;
        }
        com_password=msg_argument;
        response=com_password;
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        //printf("locked with password:");
        //printBits(sizeof(response),&response);
        return 0;
        break;
    case 1://Password Change
        if(msg_target==3){
            com_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("com password:");
            //printBits(sizeof(response),&response);
            return 0;
        }
        if(msg_argument>9999){
            response=0x20;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("wrong arg:");
            //printBits(sizeof(response),&response);
            return 1;
        }
        switch(msg_target){
        case 0:
            standby_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        case 1:
            night_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        case 2:
            alarm_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        default:
            response=0x10;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("wrong target:");
            //printBits(sizeof(response),&response);
            return 1;
            break;
        }
        break;
    case 2://Converter Values
        switch(msg_target){
        case 0:
            response=distance[0];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("ADC0:");
            //printBits(sizeof(response),&response);
            break;
        case 1:
            response=distance[1];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("ADC1:");
            //printBits(sizeof(response),&response);
            break;
        case 2:
            response=DAC_value[0];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("DAC0:");
            //printBits(sizeof(response),&response);
            break;
        case 3:
            response=DAC_value[1];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("DAC1:");
            //printBits(sizeof(response),&response);
            break;
        default:
            response=0x10;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("wrong target:");
            //printBits(sizeof(response),&response);
            return 1;
            break;
        }
        break;
    case 3://direct commands
        switch(msg_target){
        case 0:
            response=0x30;
            turn_standby();
            open_door();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        case 1:
            response=0x31;
            turn_standby();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        case 2:
            response=0x32;
            turn_alarm();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("sending back:");
            //printBits(sizeof(response),&response);
            return 0;
            break;
        default:
            response=0x10;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            //printf("wrong target:");
            //printBits(sizeof(response),&response);
            return 1;
            break;
        }
        break;
    default:
        response=0x00;
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        //printf("wrong type:");
        //printBits(sizeof(response),&response);
        return 1;
        break;
    }
    return -1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	static uint16_t ADC_Value[2];
	ADC_Value[0]=ADC_buffer[0];
    ADC_Value[1]=ADC_buffer[1];
	for(uint8_t i=0; i<2; i++){
		distance[i] = ADC_Value[i]*2000/ADC_RESOLUTION-1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){
		COM_USART_recieve();
		HAL_UART_Receive_DMA(&huart1, recieved, 3);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if((htim->Instance==TIM17)&&(overflow_flag_tim17)){
		HAL_TIM_Base_Stop_IT(&htim17);
		htim17.Instance->CNT=0;
		overflow_flag_tim17=0;
		door_status=1;
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	}
	if((htim->Instance==TIM7)&&(overflow_flag_tim7)){
		static uint8_t DAC_SPI_send_buffer[4]={0,0,0,0};
		static uint8_t DAC_SPI_ans_buffer[4]={0,0,0,0};
		static int stable_counter[2]={0,0};
		//REGULATOR//
		for(int control_iterator=0;control_iterator<2;control_iterator++){
			error[control_iterator]=motor_refference[control_iterator]-distance[control_iterator];
			if(controller_status[control_iterator]==1){
				control_value[control_iterator]=kP*error[control_iterator]+kD*(error[control_iterator]-error_prim[control_iterator]);//PD, dc motor is already an integrating object, so no need for PID
				error_prim[control_iterator]=error[control_iterator];
				if(-1*(curtain_status[control_iterator]-2)*error[control_iterator]<20){
					stable_counter[control_iterator]++;
					if(stable_counter[control_iterator]>STABLE_ITERATIONS){
						stable_counter[control_iterator]=0;
						controller_status[control_iterator]=0;
						if(curtain_status[control_iterator]==1)
							curtain_status[control_iterator]=2; else
							curtain_status[control_iterator]=0;
						if(control_iterator==0){
							HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_RESET);
						}else{
							HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_RESET);
						}
					}
				}else
					stable_counter[control_iterator]=0;
			}
			if(control_value[control_iterator]>MAX)//kontrola_zmiany
				control_value[control_iterator]=MAX;
			if(control_value[control_iterator]<MIN){
				switch(control_iterator){
				case 0:
					HAL_GPIO_WritePin(DC_MOTOR1_INV_GPIO_Port,DC_MOTOR1_INV_Pin,GPIO_PIN_SET);
					break;
				case 1:
					HAL_GPIO_WritePin(DC_MOTOR2_INV_GPIO_Port,DC_MOTOR2_INV_Pin,GPIO_PIN_SET);
					break;
				}
			}else{
				switch(control_iterator){
				case 0:
					HAL_GPIO_WritePin(DC_MOTOR1_INV_GPIO_Port,DC_MOTOR1_INV_Pin,GPIO_PIN_RESET);
					break;
				case 1:
					HAL_GPIO_WritePin(DC_MOTOR2_INV_GPIO_Port,DC_MOTOR2_INV_Pin,GPIO_PIN_RESET);
					break;
				}
			}
			DAC_value[control_iterator]=abs(control_value[control_iterator]);
		}
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port,SPI2_CS_Pin,GPIO_PIN_RESET);
		DAC_SPI_send_buffer[3]=0x03;
		DAC_SPI_send_buffer[2]=(DAC_value[0]>>12)|0x00;
		DAC_SPI_send_buffer[1]=(DAC_value[0]&0x0FF0)>>4;
		DAC_SPI_send_buffer[0]=(DAC_value[0]&0x000F)<<4;
		HAL_SPI_TransmitReceive(&hspi2, DAC_SPI_send_buffer, DAC_SPI_ans_buffer, 4, HAL_MAX_DELAY);
		DAC_SPI_send_buffer[3]=0x03;
		DAC_SPI_send_buffer[2]=(DAC_value[1]>>12)|0x01;
		DAC_SPI_send_buffer[1]=(DAC_value[1]&0x0FF0)>>4;
		DAC_SPI_send_buffer[0]=(DAC_value[1]&0x000F)<<4;
		HAL_SPI_TransmitReceive(&hspi2, DAC_SPI_send_buffer, DAC_SPI_ans_buffer, 4, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port,SPI2_CS_Pin,GPIO_PIN_RESET);
		if(controller_status[0]==0&&controller_status[1]==0){
			HAL_TIM_Base_Stop_IT(&htim7);
			htim7.Instance->CNT=0;
		}
		overflow_flag_tim7=0;
	}
}
uint8_t keypad_scan(){
	uint8_t column_masks[3]={0x6F,0xAF, 0xCF};
	uint8_t keypad_buffer;
	static uint8_t keypad_map[4][3] ={
	 { 1,  2,  3},
	 { 4,  5,  6},
	 { 7,  8,  9},
	 {0xA, 0, 0xB}
	 };
	for(int i=0; i<3; i++){
		keypad_buffer=column_masks[i];
		HAL_I2C_Mem_Write(&hi2c1, KEYPAD_WRITE_ADDR, KEYPAD_GPIOA_ADDR, 1, &keypad_buffer, sizeof(keypad_buffer), HAL_MAX_DELAY);
		HAL_Delay(3);
		HAL_I2C_Mem_Read(&hi2c1, KEYPAD_READ_ADDR, KEYPAD_GPIOA_ADDR, 1, &keypad_buffer, 1, HAL_MAX_DELAY);
		keypad_buffer=keypad_buffer & 0x0F;
		switch(keypad_buffer){
		case 0x07: return keypad_map[0][i];
			break;
		case 0x0B: return keypad_map[1][i];
			break;
		case 0x0D: return keypad_map[2][i];
			break;
		case 0x0E: return keypad_map[3][i];
			break;
		default:
			break;
		}
	}
	return 0x0C;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t code=0;
	uint8_t retries=1;
	uint8_t button=0xC;
	uint8_t rank=0;
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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,ADC_buffer,2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Receive_DMA(&huart1, recieved, 3);
  HAL_TIM_Base_Init(&htim17);//init timera
  HAL_TIM_Base_Init(&htim7);//init timera
  HAL_TIM_Base_Stop_IT(&htim7);
  HAL_TIM_Base_Stop_IT(&htim17);
  htim17.Instance->CNT=0;
  htim7.Instance->CNT=0;
  HAL_SPI_Init(&hspi2);
  HAL_I2C_Init(&hi2c1);
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
  HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0xFFFFFFFF);
  HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0xFFFFFFFF-1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
	  button=keypad_scan();
	  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin)){
	 		button=0;
			HAL_Delay(1000);
	  }else
		  button=0xC;
	  if(button!=0xC && door_status==1){
		  if(button>9){
			  rank=0;
			  code=0;
		  }else{
			  code=code+(button*(10^rank));
			  rank++;
		  }
		  if(rank==4){
			  switch(status){
			  case 0: //standby
				  if(code==standby_password){
					  open_door();
					  retries=0;
				  }else{
					  if(retries<2)
						  retries++; else{
						  turn_alarm();
						  retries=0;
					  }
				  }
				  break;
			  case 1: //alarm
				   if(code==alarm_password)
					   turn_standby();
				  break;
			  case 2: //nightmode
				  if(code==night_password){
					  turn_standby();
					  retries=0;
				  }else{
					  if(retries<2)
						  retries++; else{
						  turn_alarm();
						  retries=0;
					  }
				  }
				  break;
			  }
			  rank=0;
			  code=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM17;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 35999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 17999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 35999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 19999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI2_CS_Pin|CURTAIN1_UP_Pin|CURTAIN1_DW_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_MOTOR1_INV_Pin|DC_MOTOR2_INV_Pin|CURTAIN2_UP_Pin|CURTAIN2_DW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DCMOTOR1_ON_Pin|DCMOTOR2_ON_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin CURTAIN1_UP_Pin CURTAIN1_DW_Pin LD1_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|CURTAIN1_UP_Pin|CURTAIN1_DW_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_MOTOR1_INV_Pin DC_MOTOR2_INV_Pin CURTAIN2_UP_Pin CURTAIN2_DW_Pin */
  GPIO_InitStruct.Pin = DC_MOTOR1_INV_Pin|DC_MOTOR2_INV_Pin|CURTAIN2_UP_Pin|CURTAIN2_DW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMOTOR1_ON_Pin DCMOTOR2_ON_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DCMOTOR1_ON_Pin|DCMOTOR2_ON_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
