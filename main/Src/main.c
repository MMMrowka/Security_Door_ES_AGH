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
#define MAX 65535				//control signal is calculated as 32 bit signed int, but passed to dac as 16bit unsigned int
#define MIN 0
#define STABLE_ITERATIONS 40	//40*50ms is 2000ms=2s, if error

#define RESPONSE_BYTES 2		//size of UART response buffer
#define ADC_RESOLUTION 4096		//ADC bitsize is 12, 12^2=4096

#define KEYPAD_READ_ADDR 0x41	//these 3 values are taken from I2C expander documentation
#define KEYPAD_WRITE_ADDR 0x40
#define KEYPAD_GPIOA_ADDR 0x12


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

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
extern uint8_t overflow_flag_tim6;						//flag checked in callback function from timer (see stm32f3xx_it.c)
extern uint8_t overflow_flag_tim7;

uint16_t ADC_buffer[4]={0xFFFF,0xFFFF,0xFFFF,0xFFFF};	//purpose of buffer being of size of 4 is to check if DMA was set to right parameters in CUBE (Half word, instead of word
uint16_t distance[2]={0,0};								//distance calculated with ADC value (in mm)
uint16_t DAC_value[2]={0,0};							//value passed to DAC
int control_value[2]={0,0};								//value calculated by control algorithm before passing it to DAC
uint16_t motor_refference_prim[2]={1000,1000};			//0-open, 1000-closed
uint16_t motor_refference[2]={1000,1000};				//0-open, 1000-closed
int error[2]={0,0};										//error calculated from distance and reference values
int error_prim[2]={0,0};								//value of error in previous period of control algorithm
float kP=1;												//Value of proportional gain of the controller (placeholder)
float kD=0.5;											//Value of differentional part's gain of the controller (placeholder)

uint8_t received[3]={0,0,0};			//buffer for bytes received via USART from the supervisor PC
uint16_t com_password=0xFFFF;			//password needed to open the communication via USART from the PC
uint8_t com_locked=1;					//flag that indicates if access to supervisor commands was granted via USART

/*uint32_t dac_write_command = 0x3;
  uint32_t channel_write_ch_A = 0x0;
  uint32_t channel_write_ch_B = 0x1;*/	//currently unused

uint16_t standby_password=1234;			//keypad input needed to open the doors in standby mode
uint16_t alarm_password=2345;			//keypad input needed to go back to standby mode back from alarm mode
uint16_t night_password=3456;			//keypad input needed to go back to standby mode back from nighttime mode
uint8_t door_status=1; 					//0-opened, 1- closed
uint8_t curtain_status[2]={2,2}; 		//0-opened, 1-closing, 2 - closed, 3 - opening
uint8_t controller_status[2]={0,0};		//0-enabled, 1-disabled
uint8_t door_lock=1;					//0-open, 1-closed
uint8_t status=0;						//0-standby, 1-alarm, 2-nightmode
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
/* USER CODE BEGIN PFP */
void open_curtains(){
    if(curtain_status[0]!=0){	//if the curtain is not opened already
    	HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_SET);	//start up the dc motor
        HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_RESET);//disengage the locks..
        HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_RESET);//...that hold the curtain in place
        motor_refference[0]=0;
        curtain_status[0]=3;				//switched to opening
        controller_status[0]=1;				//enable the regulator
        HAL_TIM_Base_Start_IT(&htim7);		//start up the timer with the regulator
    }
    if(curtain_status[1]!=0){	//why not use for if both ifs are almost identical? Because of pin names in HAL functions
    	HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_RESET);
        motor_refference[1]=0;
        curtain_status[1]=3;
        controller_status[1]=1;
        HAL_TIM_Base_Start_IT(&htim7);
    }
}
void open_door(){
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);	//logic for magnetic lock is inverted to prevent it from being left opened in case of power failure
	door_status=0;					//door status is used to prevent code being input on keypad while doors are opened
	htim6.Instance->CNT=0;			//this is mostly redundant, but with timers it's "better safe than sorry" for me
	HAL_TIM_Base_Start_IT(&htim6);	//engage the door lock again in 10 minutes
};
void close_all(){		//almost the same as open_curtains()
	door_status=1;		//but doors are also closed hence "close_all()"
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);	//since I don't have a real magnetic lock LED 2 is used instead
    if(curtain_status[0]!=2){
    	HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_RESET);
        motor_refference[0]=1000;
        curtain_status[0]=1;
        controller_status[0]=1;
        HAL_TIM_Base_Start_IT(&htim7);
    }
    if(curtain_status[1]!=2){
		HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_RESET);
		motor_refference[1]=1000;
		curtain_status[1]=1;
		controller_status[1]=1;
		HAL_TIM_Base_Start_IT(&htim7);
	}
}
void turn_standby(){
	status=0;			//status is used in main logic of the program
	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET);	//turn off the alarm
    open_curtains();	//turning standby is not the same as opening the doors - that's why we open the curtains only
}
void turn_alarm(){
	status=1;
	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);		//since I don't have an alarm siren - LED 3 is used instead
    close_all();
}
void turn_nightmode(){
	status=2;			//nighttime is basically the same as alarm mode without actual alarm, it's meant to seal the room durin off hours
    close_all();
}
uint8_t* buffer_fitting(uint32_t response){			//this function takes single bitstream response and fits it into 8 bit variables to create a fitting buffer for USART
	static uint8_t buffer[RESPONSE_BYTES];
	for(int i=0;i<RESPONSE_BYTES;i++){
		buffer[i]=(response>>((RESPONSE_BYTES-1-i)*8))&0xFF;
	}
	return buffer;	//normally returning pointer to static variable within function is dangerous, but this function is used only within receive function and the buffer is used immediately after that, so there's no danger
}
int COM_USART_receive(){
    uint8_t msg_type;		//command types: 0 - open/close, 1 - password change, 2 - read control/sensor values, 3 - direct commands
    uint8_t msg_target;
    uint16_t msg_argument;					//message is split into 3 parts
    static uint16_t response;
    uint8_t* send_buffer;
    msg_type=received[0]>>4;				//first 4 bits are type of action to conduct
    msg_target=received[0]&0x0F;			//next 4 are target of the message
    msg_argument=received[1]<<8;			//last two are arguments
    msg_argument=msg_argument|received[2];	//argument is applicable to password setting
    if(msg_type==0&&msg_target==0&&com_locked==1){	//if com is locked password needs to be sent first to unlock it
        if(msg_argument==com_password)	//if password is correct
        {
            com_locked=0;	//unlock the comm
            response=0;		//send 0
            send_buffer=buffer_fitting(response);
            HAL_UART_Transmit_DMA(&huart1, send_buffer, 2); //this could be integrated into buffer fitting as a send() function, but I get a feeling it's better to place it here for clarity
            return 0;										//if we had other USART based comm I would be in favour of doing that ^, because it'd remove the issue of returning pointer to static variable declared in function.
        }else{
            com_locked=1;
            response=1;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 0;
        }
    }
    if(com_locked==1){		//if comm is locked - access is denied to everything that's not a password guess.
        response=0xF1;
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        return 1;
    }
    switch(msg_type){
    case 0://close comm
    	com_locked=1;
        if(msg_target!=1){	//since we only have two options if is as applicable as switch-case.
            response=0x01;	//if message is not of target 0 (lock with password), old password is used
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 0;
        }
        com_password=msg_argument;	//if message is of target 0 - argument is used as a new comm password
        response=com_password;
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        return 0;
        break;
    case 1://Password Change
        if(msg_target==3){		//we can also change comm password in this way, if is used because comm password unlike keypad passwords is not limited to 9999
            com_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 0;
        }
        if(msg_argument>9999){	//keypadd passwords are 4 decimal digits so they can't exceed 9999
            response=0x30;		//0x30 = wrong argument.
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 1;
        }
        switch(msg_target){		//there is really nothing to say here. it's just changing keypad codes
        case 0:					//standby code (door opening)
            standby_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 1:					//night code (switching from nighttime to standby)
            night_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 2:					//alarm code (switching from alarm to standby)
            alarm_password=msg_argument;
            response=msg_argument;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        default:
            response=0x20;	//0x20 - wrong target
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 1;	//error was detected (or access was denied)
        }
        return 0;		//no error was detected
    case 2:						//Converter Values
        switch(msg_target){ 	//we use this to return values read by adc and written to dac
        case 0:					//distance from 1st sensor
            response=distance[0];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 1:					//distance from 2nd sensor
            response=distance[1];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 2:					//value sent to first channel of DAC
            response=DAC_value[0];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 3:
            response=DAC_value[1];
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        default:
            response=0x20;	//wrong target
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 1;
        }
        return 0;
    case 3:					//direct commands
        switch(msg_target){
        case 0:				//open all command - turns standby (opening curtains) and opens the door
        					//THIS HAS TO BE USED AFTER LAUNCHING THE SYSTEM since the initial configuration (closed curtains, standby mode, closed door is unobtainable by working system on it's own
        					//this is by design to ensure the security system is always supervised by an overwatch PC unit
            response=0x03;	//03 and not 30 not to confuse it with "wrong argument"
            turn_standby();
            open_door();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 1:				//turn alarm/nightmode off/turn standby - this is applicable where someone locks the operator inside.
            response=0x13;
            turn_standby();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        case 2:				//turn alarm on - this is used when safety of an operator is imperiled and he has to close the door
            response=0x23;
            turn_alarm();
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            break;
        default:
            response=0x20;
            send_buffer=buffer_fitting(response);
			HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
            return 1;
        }
        return 0;
    default:
        response=0x10;	//0x10 = wrong type
        send_buffer=buffer_fitting(response);
		HAL_UART_Transmit_DMA(&huart1, send_buffer, 2);
        return 1;
    }
    return 2;	//unknown error (there's no way the program would get here)
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	for(uint8_t i=0; i<2; i++){
		distance[i] = ADC_buffer[i]*2000/ADC_RESOLUTION-1;	//2000 mm is the maximum range of the sensor (as we can read in the documentation)
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){
		COM_USART_receive();								//call the received message processing function
		HAL_UART_Receive_DMA(&huart1, received, 3);			//set the USART back to receive
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if((htim->Instance==TIM6)&&(overflow_flag_tim6)){	//this 10 second timer will engage the magnetic lock after disengaging them by open_door()
		HAL_TIM_Base_Stop_IT(&htim6);					//shutting the timer off till it's enabled again by door opening
		htim6.Instance->CNT=0;
		overflow_flag_tim6=0;
		door_status=1;
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	}
	if((htim->Instance==TIM7)&&(overflow_flag_tim7)){	//PD regulator
		static uint8_t DAC_SPI_send_buffer[4]={0,0,0,0};
		static uint8_t DAC_SPI_ans_buffer[4]={0,0,0,0};	//DAC buffers for SPI transmiting function
		static int stable_counter[2]={0,0};
		//REGULATOR//
		for(int control_iterator=0;control_iterator<2;control_iterator++){
			error[control_iterator]=motor_refference[control_iterator]-distance[control_iterator];	//calculating error
			if(controller_status[control_iterator]==1){									//if regulator is enabled
				//PD, dc motor is already an integrating object, so no need for PID
				control_value[control_iterator]=kP*error[control_iterator]+kD*(error[control_iterator]-error_prim[control_iterator]);
				error_prim[control_iterator]=error[control_iterator];					//setting current error as past error for the next period
				if(abs(error[control_iterator])<20){	//this might seem confusing, when opening - error value is reference(0)-distance(positive) so = negative, when closing - it's positive, this statement ensures that we always compare 20 with absolute value of error
					stable_counter[control_iterator]++;						//if the value of error is less than 2cm (in range for lock to be able to hold the curtain) it counts as stable iteration
					if(stable_counter[control_iterator]>STABLE_ITERATIONS){	//if curtains are in lock range for more than 2 seconds controller is disabled and locks are engaged
						stable_counter[control_iterator]=0;					//disabling controller
						controller_status[control_iterator]=0;				//resetting counter of iterations
						if(curtain_status[control_iterator]==1)				//if doors are closing...
							curtain_status[control_iterator]=2; else		//...set as closed
							curtain_status[control_iterator]=0;				//if opening - set as open
						if(control_iterator==0){							//can't use for here, because of pin names
							HAL_GPIO_WritePin(CURTAIN1_UP_GPIO_Port,CURTAIN1_UP_Pin,GPIO_PIN_SET);//first engage the locks - then reset the motor
							HAL_GPIO_WritePin(CURTAIN1_DW_GPIO_Port,CURTAIN1_DW_Pin,GPIO_PIN_SET);
							HAL_Delay(20); 	//this is to ensure the curtains won't move before locks are engaged.
							HAL_GPIO_WritePin(DCMOTOR1_ON_GPIO_Port,DCMOTOR1_ON_Pin,GPIO_PIN_RESET);
						}else{
							HAL_GPIO_WritePin(CURTAIN2_UP_GPIO_Port,CURTAIN2_UP_Pin,GPIO_PIN_SET);
							HAL_GPIO_WritePin(CURTAIN2_DW_GPIO_Port,CURTAIN2_DW_Pin,GPIO_PIN_SET);
							HAL_Delay(20);
							HAL_GPIO_WritePin(DCMOTOR2_ON_GPIO_Port,DCMOTOR2_ON_Pin,GPIO_PIN_RESET);
						}
					}
				}else
					stable_counter[control_iterator]=0;		//if curtain moves out of lock range even for an iteration - counter is reset
			}
			if(control_value[control_iterator]>MAX)			//as stated before - we need to make sure we won't overflow the 16 unsigned int with 32 bit int value
				control_value[control_iterator]=MAX;
			if(control_value[control_iterator]<MIN){
				switch(control_iterator){	//the part that starts here
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
					break;					//and ends here ensures that in case of value being negative - the dc motor will work in other direction (this can also be done within the physical circuit)
				}
			}
			DAC_value[control_iterator]=abs(control_value[control_iterator]);	//we're inverting the work of motor in code - we convert the abs value
		}
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port,SPI2_CS_Pin,GPIO_PIN_RESET);		//chip select is in reset - slave is listening
		DAC_SPI_send_buffer[3]=0x03;						//from documentation - write and update DAC channel
		DAC_SPI_send_buffer[2]=(DAC_value[0]>>12)|0x00;		//selecting 1st DAC channel
		DAC_SPI_send_buffer[1]=(DAC_value[0]&0x0FF0)>>4;	//fitting the value to be converted
		DAC_SPI_send_buffer[0]=(DAC_value[0]&0x000F)<<4;
		HAL_SPI_TransmitReceive(&hspi2, DAC_SPI_send_buffer, DAC_SPI_ans_buffer, 4, HAL_MAX_DELAY);
		DAC_SPI_send_buffer[3]=0x03;						//same procedure for 2nd DAC channel
		DAC_SPI_send_buffer[2]=(DAC_value[1]>>12)|0x01;
		DAC_SPI_send_buffer[1]=(DAC_value[1]&0x0FF0)>>4;
		DAC_SPI_send_buffer[0]=(DAC_value[1]&0x000F)<<4;
		HAL_SPI_TransmitReceive(&hspi2, DAC_SPI_send_buffer, DAC_SPI_ans_buffer, 4, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port,SPI2_CS_Pin,GPIO_PIN_RESET);
		if(controller_status[0]==0&&controller_status[1]==0){	//if both curtains are in place and regulators are disabled - stop the timer
			HAL_TIM_Base_Stop_IT(&htim7);
			htim7.Instance->CNT=0;
		}
		overflow_flag_tim7=0;
	}
}
uint8_t keypad_scan(){							//standard keypad scanning function
	uint8_t column_masks[3]={0x6F,0xAF, 0xCF};	//those masks serve to set appropiate column bits at keypad to low, while setting the rest of the column and row bits to high
	uint8_t keypad_buffer;
	static uint8_t keypad_map[4][3] ={			//keypad map used for clarity
	 { 1,  2,  3},
	 { 4,  5,  6},
	 { 7,  8,  9},
	 {0xA, 0, 0xB}
	 };
	for(int i=0; i<3; i++){
		keypad_buffer=column_masks[i];	//we iterate through column masks
		HAL_I2C_Mem_Write(&hi2c1, KEYPAD_WRITE_ADDR, KEYPAD_GPIOA_ADDR, 1, &keypad_buffer, sizeof(keypad_buffer), HAL_MAX_DELAY);
										//line above ensures all bits but the column we're scanning are set to 1
		HAL_Delay(3);					//this short interval is to ensure all pins are updated
		HAL_I2C_Mem_Read(&hi2c1, KEYPAD_READ_ADDR, KEYPAD_GPIOA_ADDR, 1, &keypad_buffer, 1, HAL_MAX_DELAY);
										//this line reads all the column and row pins' values
		keypad_buffer=keypad_buffer & 0x0F;	//if any of the buttons in the column was pressed - it's row will also be pulled to 0
		switch(keypad_buffer){				//if any of the row bits are 0 this means button in this row and currently scanned column was pressed
		case 0x07:
			return keypad_map[0][i];
			break;
		case 0x0B:
			return keypad_map[1][i];
			break;
		case 0x0D:
			return keypad_map[2][i];
			break;
		case 0x0E:
			return keypad_map[3][i];
			break;
		default:
			break;
		}
	}
	return 0x0C;							//if none of the buttons was pressed - return a number that's not on the keypad
}	//this function is really the most standard way of doing it and could be implemented with GPIO pins on the board, using expander is not that necessary unless we're short on available gpio outputs
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
	uint16_t code=0;		//code input on keypad
	uint8_t retries=0;		//available number of retries for inputting the password in standby mode
	uint8_t button=0xC;		//keypad_scan() output
	uint8_t rank=0;			//rank of the decimal variable added to code
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
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,ADC_buffer,2);		//inits and starts are pretty self explanatory
  HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Receive_DMA(&huart1, received, 3);
  HAL_TIM_Base_Init(&htim6);
  HAL_TIM_Base_Init(&htim7);
  HAL_TIM_Base_Stop_IT(&htim7);					//just to make sure timer won't stop untill we want it to
  HAL_TIM_Base_Stop_IT(&htim6);
  htim6.Instance->CNT=0;						//...and that it will start from 0
  htim7.Instance->CNT=0;						//Just as I said - it's "better safe than sorry with timers
  HAL_SPI_Init(&hspi2);
  HAL_I2C_Init(&hi2c1);
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);	//those 3 lines are just to make sure doors are really closed after init
  HAL_Delay(50);
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);			//this delay is to prevent user from putting in two characters on keypad instead of one (happens to me a lot when I try to enter my block)
	  button=keypad_scan();		//scanning the keypad
	  if(button!=0xC && door_status==1){	//if a button was pressed and doors are not opened at the moment.
		  if(button>9){						//two extra buttons on keypad serve as reset
			  rank=0;
			  code=0;
		  }else{
			  code=code+(button*(10^rank));	//pressed button is added to 4 digit code as a next digit
			  rank++;						//rank is decimal rank of keypad code
		  }
		  if(rank==4){						//if 4 digits were put in
			  switch(status){
			  case 0: 	//standby
				  if(code==standby_password){	//if password is correct - open the door
					  open_door();
					  retries=0;
				  }else{
					  if(retries<2)				//if it's not - add an attempt
						  retries++; else{		//after 3 unsuccessful attempts - sound the alarm
						  turn_alarm();
						  retries=0;
					  }
				  }
				  break;
			  case 1: 							//alarm
				   if(code==alarm_password)		//if right alarm code was put in - shut off the alarm
					   turn_standby();			//there is no retries here, because how would it work? what state would it go after 3 attempts? Doublealarm?
				  break;
			  case 2: 							//nightmode
				  if(code==night_password){		//nightmode is just like standby (it also has retries)
					  turn_standby();			//the only diffrence is that it opens curtains (turns standby) and doesn't open the door
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
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0;
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
