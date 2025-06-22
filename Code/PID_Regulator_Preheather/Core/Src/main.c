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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MAX31855.h"
#include "stdio.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
// variables
int maximum_firing_delay = 99;
/*Later in the code you will se that the maximum delay after the zero detection
 * is 7400. Why? Well, we know that the 220V AC voltage has a frequency of around 50-60HZ so
 * the period is between 20ms and 16ms, depending on the country. We control the firing
 * delay each half period so each 10ms or 8 ms. To amke sure we wont pass thsoe 10ms, I've made tests
 * and the 7400us or 7.4ms was a good value. Measure your frequency and chande that value later */
//////////////////////////////////////////////////////
int firing_delay = 100;
int last_Error = 0;
bool zero_cross_detected = false;
float real_temperature_heater = 0;
float real_temperature_element = 0;
float real_temperature_heater_last = 0;
float real_temperature_element_last = 0;
float set_temperature = 99;
float selected_temperature = 0;
int temp_select = 1;
int mode_select = 1;
int menu = 0;
float power = 0;
float power_last = 100;
float pid_output_max = 100.0;
float pid_output_min = 0.0;
float i_term_max = 50.0;
bool button = false;
char temp[10];
int sens_error_HT = 0;
int sens_error_IC = 0;
int sens_error = 0;
int turn_CW_CCW = 0; // 1 left 2 right
int menu_last = 100;
bool sub_menu = false;
int sub_menu_pos = 0;
int sub_menu_pos_last = 100;
int set_temperature_last = 100;
bool temp_set = false;
uint8_t PC_TX[20];
uint8_t PC_RX[20];
bool TX_state = true;
// PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, timePrev, time_stamp;
volatile uint32_t Time = 0;
volatile uint32_t uart_time = 0;
volatile uint32_t temp1_time = 0;
volatile uint32_t temp2_time = 0;
int PID_value = 0;
volatile uint8_t set = 0;
// PID constants //TODO do ustawienia
float kp = 16.8;
float ki = 0.2836; //0.00006598; // 0.6598;
float kd = 248.85; //54.5985;
int PID_p = 0;
float PID_i = 0;
float PID_d = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void){

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
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	ssd1306_Init();
	ssd1306_SetCursor(30,20);
	ssd1306_WriteString("LOGO",Font_16x26,White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_DMA(&huart1,PC_RX,2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while(1){ // TODO zmienic i2c na interrupt
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		sens_error = 0;
		real_temperature_heater = Max31855_Read_Temp(1);
		if(((real_temperature_heater - real_temperature_heater_last) > 20 || (real_temperature_heater - real_temperature_heater_last) < -20) && real_temperature_heater_last != 0){
			real_temperature_heater = real_temperature_heater_last;
		}
		sens_error_HT = Error; // Directly assign error status for heater sensor

		real_temperature_element = Max31855_Read_Temp(2); // TODO dodac lepsza obsluge bledu
		if(((real_temperature_element - real_temperature_element_last) > 20 || (real_temperature_element - real_temperature_element_last) < -20) && real_temperature_element_last != 0){
			real_temperature_element = real_temperature_element_last;
		}
		sens_error_IC = Error;					// Directly assign error status for element sensor

		if(sens_error_HT != 0){
			sens_error = sens_error_HT;
		}
		else{ // If heater sensor is OK, use element sensor status (could be error or OK)
			sens_error = sens_error_IC;
		}
		if(sens_error == 0 && last_Error != sens_error){
			menu_last = 100;
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			last_Error = sens_error;
		}

		if(last_Error != sens_error && sens_error != 0){
			switch(sens_error){
				case 1:
					ssd1306_Fill(Black);
					ssd1306_UpdateScreen();
					ssd1306_SetCursor(20,5);
					ssd1306_WriteString("Error",Font_16x26,White);
					ssd1306_SetCursor(5,28);
					if(sens_error_IC != 0){
						ssd1306_WriteString("Sensor IC",Font_11x18,White);
					}
					else if(sens_error_HT != 0){
						ssd1306_WriteString("Sensor HEAT",Font_11x18,White);
					}
					ssd1306_SetCursor(15,48);
					ssd1306_WriteString("NOT CONNECTED",Font_7x10,White);
					ssd1306_UpdateScreen();
				break;
				case 2:
					ssd1306_Fill(Black);
					ssd1306_UpdateScreen();
					ssd1306_SetCursor(20,5);
					ssd1306_WriteString("Error",Font_16x26,White);
					ssd1306_SetCursor(5,28);
					if(sens_error_IC != 0){
						ssd1306_WriteString("Sensor IC",Font_11x18,White);
					}
					else if(sens_error_HT != 0){
						ssd1306_WriteString("Sensor HEAT",Font_11x18,White);
					}
					ssd1306_SetCursor(15,48);
					ssd1306_WriteString("SHORT TO GND",Font_7x10,White);
					ssd1306_UpdateScreen();
				break;
				case 4:
					ssd1306_Fill(Black);
					ssd1306_UpdateScreen();
					ssd1306_SetCursor(20,5);
					ssd1306_WriteString("Error",Font_16x26,White);
					ssd1306_SetCursor(15,28);
					if(sens_error_IC != 0){
						ssd1306_WriteString("Sensor IC",Font_11x18,White);
					}
					else if(sens_error_HT != 0){
						ssd1306_WriteString("Sensor HEAT",Font_11x18,White);
					}
					ssd1306_SetCursor(15,48);
					ssd1306_WriteString("SHORT TO VCC",Font_7x10,White);
					ssd1306_UpdateScreen();
				break;
				default:
				break;
			}
			last_Error = sens_error;
		}

		if(sens_error == 0){ // TODO dodac wyświetlanie która termopara jest aktualnie używana
			if(real_temperature_element_last != real_temperature_element || real_temperature_heater_last != real_temperature_heater){ // TODO zmienic na wysylanie tylko zmienionej liczby
				ssd1306_SetCursor(5,5);
				ssd1306_WriteString("IC:",Font_7x10,White);
				for(int i = 0; i < 9; i++){
					temp[i] = 0;
				}
				if(temp_select == 2){
					sprintf(temp,"%.0f X  ",real_temperature_element);
				}
				else{
					sprintf(temp,"%.0f   ",real_temperature_element);
				}
				ssd1306_SetCursor(27,5);
				ssd1306_WriteString(temp,Font_7x10,White);
				ssd1306_SetCursor(70,5);
				ssd1306_WriteString("HT:",Font_7x10,White);
				for(int i = 0; i < 9; i++){
					temp[i] = 0;
				}
				if(temp_select == 1){
					sprintf(temp,"%.0f X ",real_temperature_heater);
				}
				else{
					sprintf(temp,"%.0f  ",real_temperature_heater);
				}
				ssd1306_SetCursor(92,5);
				ssd1306_WriteString(temp,Font_7x10,White);
				ssd1306_UpdateScreen();
				real_temperature_element_last = real_temperature_element;
				real_temperature_heater_last = real_temperature_heater;
			}
			if(temp_set == 1){
				if(mode_select == 1){
					if(set_temperature != set_temperature_last){
						ssd1306_FillRectangle(54,15,75,25,Black);
						ssd1306_UpdateScreen();
						ssd1306_SetCursor(5,15);
						ssd1306_WriteString("Set:",Font_7x10,White);
						for(int i = 0; i < 9; i++){
							temp[i] = 0;
						}
						sprintf(temp,"%.0f Auto",set_temperature);
						ssd1306_SetCursor(34,15);
						ssd1306_WriteString(temp,Font_7x10,White);
						ssd1306_UpdateScreen();
					}
				}
				else if(mode_select == 2){
					if(power != power_last){
						ssd1306_FillRectangle(54,15,75,25,Black);
						ssd1306_UpdateScreen();
						ssd1306_SetCursor(5,15);
						ssd1306_WriteString("Set:",Font_7x10,White);
						for(int i = 0; i < 9; i++){
							temp[i] = 0;
						}
						sprintf(temp,"%.0f% Manual",power);
						ssd1306_SetCursor(34,15);
						ssd1306_WriteString(temp,Font_7x10,White);
						ssd1306_UpdateScreen();
					}
				}
			}
			else if(temp_set == 0){
				ssd1306_SetCursor(5,15);
				ssd1306_WriteString("Heater OFF",Font_7x10,White);
				ssd1306_UpdateScreen();
			}
			if(!sub_menu){
				if(turn_CW_CCW == 2){
					menu++;
					if(menu > 3){
						menu = 0;
					}
				}
				else if(turn_CW_CCW == 1){
					menu--;
					if(menu < 0){
						menu = 3;
					}
				}
				turn_CW_CCW = 0;
				if(menu_last != menu){
					switch(menu){
						case 0:
							ssd1306_FillRectangle(5,25,128,64,Black);
							ssd1306_UpdateScreen();
							ssd1306_SetCursor(5,25);
							ssd1306_WriteString("Source",Font_11x18,Black);
							ssd1306_SetCursor(5,45);
							ssd1306_WriteString("MAN/AUTO",Font_11x18,White);
							ssd1306_UpdateScreen();
						break;
						case 1:
							ssd1306_FillRectangle(5,25,128,64,Black);
							ssd1306_UpdateScreen();
							ssd1306_SetCursor(5,25);
							ssd1306_WriteString("MAN/AUTO",Font_11x18,Black);
							ssd1306_SetCursor(5,45);
							if(mode_select == 1){
								ssd1306_WriteString("Temperature",Font_11x18,White);
							}
							else if(mode_select == 2){
								ssd1306_WriteString("Power",Font_11x18,White);
							}
							ssd1306_UpdateScreen();
						break;
						case 2:
							ssd1306_FillRectangle(5,25,128,64,Black);
							ssd1306_UpdateScreen();
							ssd1306_SetCursor(5,25);
							if(mode_select == 1){
								ssd1306_WriteString("Temperature",Font_11x18,Black);
							}
							else if(mode_select == 2){
								ssd1306_WriteString("Power",Font_11x18,Black);
							}
							ssd1306_SetCursor(5,45);
							ssd1306_WriteString("Heat ON/OFF",Font_11x18,White);
							ssd1306_UpdateScreen();
						break;
						case 3:
							ssd1306_FillRectangle(5,25,128,64,Black);
							ssd1306_UpdateScreen();
							ssd1306_SetCursor(5,25);
							ssd1306_WriteString("Heat ON/OFF",Font_11x18,Black);
							ssd1306_SetCursor(5,45);
							ssd1306_WriteString("Source",Font_11x18,White);
							ssd1306_UpdateScreen();
						break;
						default:

						break;
					}
					menu_last = menu;
				}
				if(button == true){
					sub_menu = true;
					button = false;
				}
			}

			if(sub_menu){
				if(menu != 2){
					if(turn_CW_CCW == 2){
						sub_menu_pos++;
						if(sub_menu_pos > 1){
							sub_menu_pos = 0;
						}
					}
					else if(turn_CW_CCW == 1){
						sub_menu_pos--;
						if(sub_menu_pos < 0){
							sub_menu_pos = 1;
						}
					}
					turn_CW_CCW = 0;
				}
				if(sub_menu_pos_last != sub_menu_pos || set_temperature != set_temperature_last || (mode_select == 2 && power != power_last)){
					switch(menu){
						case 0:
							switch(sub_menu_pos){
								case 0:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Element",Font_11x18,Black);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("Heater",Font_11x18,White);
									ssd1306_UpdateScreen();
								break;
								case 1:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Heater",Font_11x18,Black);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("Element",Font_11x18,White);
									ssd1306_UpdateScreen();
								break;
								default:
								break;
							}
						break;
						case 1:
							switch(sub_menu_pos){
								case 0:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Manual",Font_11x18,Black);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("Auto",Font_11x18,White);
									ssd1306_UpdateScreen();
								break;
								case 1:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Auto",Font_11x18,Black);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("Manual",Font_11x18,White);
									ssd1306_UpdateScreen();
								break;
								default:
								break;
							}
						break;
						case 2:

							ssd1306_FillRectangle(5,25,128,64,Black);
							ssd1306_UpdateScreen();
							for(int i = 0; i < 9; i++){
								temp[i] = 0;
							}
							if(mode_select == 1){
								sprintf(temp,"%.0f  ",set_temperature);
								ssd1306_SetCursor(5,25);
								ssd1306_WriteString("Temp",Font_11x18,White);
								ssd1306_SetCursor(5,45);
								if(set_temperature < 100){
									ssd1306_WriteString("Off",Font_11x18,White);
								}
								else if(set_temperature >= 100){
									ssd1306_WriteString(temp,Font_11x18,Black);
								}
							}
							else if(mode_select == 2){
								sprintf(temp,"%.0f",power);
								ssd1306_SetCursor(5,25);
								ssd1306_WriteString("Power",Font_11x18,White);
								ssd1306_SetCursor(5,45);
								if(power < 0){
									ssd1306_WriteString("Off",Font_11x18,White);
								}
								else if(power >= 0){
									ssd1306_WriteString(temp,Font_11x18,Black);
								}
							}
							ssd1306_UpdateScreen();

						break;
						case 3:
							switch(sub_menu_pos){
								case 0:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Turn off?",Font_11x18,White);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("Yes",Font_11x18,Black);
									ssd1306_UpdateScreen();
								break;
								case 1:
									ssd1306_FillRectangle(5,25,128,64,Black);
									ssd1306_UpdateScreen();
									ssd1306_SetCursor(5,25);
									ssd1306_WriteString("Turn off?",Font_11x18,White);
									ssd1306_SetCursor(5,45);
									ssd1306_WriteString("No",Font_11x18,Black);
									ssd1306_UpdateScreen();
								break;
								default:
								break;
							}
						break;
						default:

						break;
					}
					sub_menu_pos_last = sub_menu_pos;
					set_temperature_last = set_temperature;
					power_last = power;
				}
				if(menu == 2){
					if(mode_select == 1){
						if(turn_CW_CCW == 2){
							set_temperature++;
							if(set_temperature >= 400){
								set_temperature = 400;
							}
						}
						else if(turn_CW_CCW == 1){
							set_temperature--;
							if(set_temperature <= 99){
								set_temperature = 99;
							}
						}
						turn_CW_CCW = 0;
					}
					if(mode_select == 2){
						if(turn_CW_CCW == 2){
							power = power + 10;
							if(power >= 100){
								power = 100;
							}
						}
						else if(turn_CW_CCW == 1){
							power = power - 10;
							if(power <= 0){
								power = 0;
							}
						}
						turn_CW_CCW = 0;
					}
				}
				if(button == true){
					sub_menu = false;
					button = false;
					switch(menu){
						case 0:
							switch(sub_menu_pos){
								case 0:
									temp_select = 2;
								break;
								case 1:
									temp_select = 1;
								break;
								default:
								break;
							}
						break;
						case 1:
							switch(sub_menu_pos){
								case 0:
									mode_select = 2;
								break;
								case 1:
									mode_select = 1;
								break;
								default:
								break;
							}
						break;
						case 2:
							if(mode_select == 1){
								if(set_temperature >= 100){
									temp_set = true;
								}
								else if(set_temperature <= 99){
									temp_set = false;
									PID_value = 0;
									power = 0;
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
								}
							}
							else if(mode_select == 2){
								if(power > 0){
									temp_set = true;
								}
								else if(power <= 0){
									temp_set = false;
									PID_value = 0;
									power = 0;
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
								}
							}
						break;
						case 3:
							switch(sub_menu_pos){
								case 0:
									temp_set = false;
									set_temperature = 99;
									PID_value = 0;
									power = 0;
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
								break;
								case 1:

								break;
								default:
								break;
							}

						break;
					}
					menu_last = 100;
					sub_menu_pos_last = 100;
					set_temperature_last = 100;
					power_last = 100;
				}
			}
		}
		if(temp_select == 1){
			selected_temperature = real_temperature_heater;
		}
		else if(temp_select == 2){
			selected_temperature = real_temperature_element;
		}
		if(PC_RX[1] == 49){
			temp_set = false;
			set_temperature = 99;
			PID_value = 0;
			power = 0;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		if(temp_set){
			if(mode_select == 1){
				// --- OBLICZENIA ---
				PID_error = set_temperature - selected_temperature;

// --- Czas ---
				elapsedTime = (float)(Time - timePrev) / 10000.0f; // Time w krokach 100 µs => dzielimy przez 10k = sekundy
				timePrev = Time;

// --- PID: P ---
				PID_p = kp * PID_error;

// --- PID: I (z zabezpieczeniem przed windupem) ---
				PID_i += ki * PID_error * elapsedTime;
				if(PID_i > i_term_max)
					PID_i = i_term_max;
				if(PID_i < -i_term_max)
					PID_i = -i_term_max;

// --- PID: D ---
				PID_d = 0;
				if(elapsedTime > 0){
					PID_d = kd * ((PID_error - previous_error) / elapsedTime);
				}
				previous_error = PID_error;

// --- SUMA PID jako "procent mocy" ---
				power = PID_p + PID_i + PID_d;

// --- Ograniczenie mocy 0–100% ---
				if(power > pid_output_max){
					power = pid_output_max;
				}
				if(power < pid_output_min){
					power = pid_output_min;
				}
			}
// --- KONWERSJA mocy (%) na opóźnienie triaka ---

			PID_value = maximum_firing_delay * (power / 100.0); // im większa moc, tym mniejsze opóźnienie

// --- Zabezpieczenie ---
			if(PID_value < 0.0){
				PID_value = 0.0;
			}
			if(PID_value > maximum_firing_delay){
				PID_value = maximum_firing_delay;
			}
		}

		if((uint32_t)Time - uart_time >= 500){
			for(int i = 0; i < sizeof(PC_TX); i++){
				PC_TX[i] = 0;
			}
			snprintf((char*)PC_TX,sizeof(PC_TX),"%.0f,%.0f,%d,%d,%.0f\n",real_temperature_element,real_temperature_heater,sens_error_IC,sens_error_HT,power);
			uart_time = Time;
			if(TX_state == 1){
				HAL_UART_Transmit_DMA(&huart1,PC_TX,(uint16_t)strlen((char*)PC_TX));
				TX_state = false;
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_1) != HAL_OK){
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void){

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x0090194B;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&hi2c2) != HAL_OK){
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if(HAL_I2CEx_ConfigAnalogFilter(&hi2c2,I2C_ANALOGFILTER_ENABLE) != HAL_OK){
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if(HAL_I2CEx_ConfigDigitalFilter(&hi2c2,0) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void){

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if(HAL_SPI_Init(&hspi1) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void){

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 4799;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if(HAL_TIM_Base_Init(&htim1) != HAL_OK){
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim1,&sClockSourceConfig) != HAL_OK){
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim1,&sMasterConfig) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void){

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
	if(HAL_UART_Init(&huart1) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void){

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_15,GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

	/*Configure GPIO pins : PA1 PA4 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn,3,0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_3_IRQn,3,0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_15_IRQn,3,0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		if(temp_set){
			set = 1;
			time_stamp = Time;
			//			while((uint32_t)(Time - time_stamp) <= (maximum_firing_delay - PID_value)); //FIXME gdy duzy delay to nie wykonuje petli glownej dodac moze timer ktory po zadanym czasie wlaczy pin
			//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); //FIXME dlugo czeka żeby wykonac to przez petle glowna
			//			time_stamp = Time;
			//			while((uint32_t)(Time - time_stamp) <= 100);
			//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		//		zero_cross_detected = true;
	}
	else if(GPIO_Pin == GPIO_PIN_2){
		button = true;
		HAL_Delay(10);
	}
	else if(GPIO_Pin == GPIO_PIN_8){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)){
			turn_CW_CCW = 1;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_9){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)){
			turn_CW_CCW = 2;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	Time++;
	if(PID_value != 0){
		if(set == 1){
			if((uint32_t)(Time - time_stamp) >= ((maximum_firing_delay - PID_value) + 10)){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				time_stamp = Time;
				set = 2;
			}
		}
		if(set == 2){
			if((uint32_t)(Time - time_stamp) >= 1){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				set = 0;
			}
		}
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
	TX_state = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_DMA(&huart1,PC_RX,2);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void){
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while(1){
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
