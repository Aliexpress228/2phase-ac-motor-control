/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <locale.h>


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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t ADCdata;
int16_t error = 0;
int16_t p_error = 0;



volatile float angl1 = 0.0f;
volatile float angl2 = 90.0f;
volatile float angl3 = 0.0f;
volatile float angl4 = 0.0f;


float freq = 1;
float RPM = 0;
float Tpp = 10;
float speed = 0;
float angular_speed_rad_per_sec = 0;

int speed_desired = 10;
int angle_desired = 30*360;

float pid_integral_speed = 0;
float prev_speed_error = 0;

float integral = 0.0f;
float prev_error = 0.0f;
float pid_output_speed =0;
uint32_t last_time = 0;
uint8_t stop_flag = 0;



volatile int counter1 = 0;
volatile int counter2 = 0;

#define handler_prescaler 1200
#define handler_scaler_for_adc 40
#define PI 3.14159265359
#define PWM_PERIOD 2000
#define SINE_STEPS 360
#define TABLE_SIZE 18
#define ADC_AVG_COUNT 3
#define RPM_MAX 10000
#define MIN_FREQ 0.0f  // Минимальная частота
#define MAX_FREQ 750.0f // Максимальная частота
#define MAX_FREQ_ADJUSTMENT 200.0f // Максимальное изменение за шаг
#define KP 0.05f           // Коэффициент регулятора
#define KI 0.001f        // �?нтегральный коэффициент
#define KD 0.0f

#define KP2 0.005f           // Коэффициент регулятора
#define KI2 0.0001f        // �?нтегральный коэффициент
#define KD2 0.0f

uint32_t adc_values[ADC_AVG_COUNT] = {0};
uint8_t adc_index = 0;
uint32_t adc_sum = 0;

#define ADC_TABLE_SIZE 40

int16_t speed_table[ADC_TABLE_SIZE] = {
  0, 103, 159, 187, 218, 243, 269, 290, 314, 336, 357,
  374, 401, 429, 445, 473, 486, 510, 534, 558, 579,
  595, 622, 641, 665, 693, 717, 740, 751, 781, 804,
  834, 850, 872, 936, 949, 972, 1012, 1052, 1175
};

int16_t adc_table[ADC_TABLE_SIZE] = {
  0, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100,
  1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100,
  2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100,
  3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000
};

double sinuses[360];

int n = 15;

uint32_t adc_value = 0;
uint32_t voltage_mv = 0;
uint32_t voltage_avg_mv=0;

uint32_t prev_adc_value=0;
uint8_t derivative_sign=1;
uint32_t last_extremum_time;
int shaft_angle_deg=0;

char buffer[50];
uint32_t button_last_time = 0;
const uint32_t button_debounce = 100;
uint32_t adc_update_flag = 0;

uint32_t adc2_raw_value = 0;
uint32_t adc2_phase = 0;
int  derivative=0;
#define DERIVATIVE_WINDOW_SIZE 20
int derivative_buffer[DERIVATIVE_WINDOW_SIZE] = {0};
int derivative_buffer_index = 0;
int derivative_sum = 0;
float pid_integral = 0;
float pid_prev_error = 0;
#define MEDIAN_WINDOW_SIZE 25
static float speed_buffer[MEDIAN_WINDOW_SIZE] = {0};
static int sorted_indices[MEDIAN_WINDOW_SIZE] = {0};
static int position_in_sorted[MEDIAN_WINDOW_SIZE] = {0};
static int buffer_index = 0;
static int samples_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SWO_Print(char* msg) {
    for (int i = 0; msg[i] != '\0'; i++) {
        ITM_SendChar(msg[i]);
    }
    ITM_SendChar('\r');
    ITM_SendChar('\n');
}

float getSpeedFromADC(float adc) {
    for (int i = 0; i < ADC_TABLE_SIZE - 1; i++) {
        if (adc >= adc_table[i] && adc <= adc_table[i + 1]) {
            float x0 = adc_table[i];
            float x1 = adc_table[i + 1];
            float y0 = speed_table[i];
            float y1 = speed_table[i + 1];

            // Линейная интерполяция
            return y0 + ((adc - x0) * (y1 - y0)) / (x1 - x0);
        }
    }

    // Обработка вне диапазона
    if (adc < adc_table[0])
        return speed_table[0];
    else
        return speed_table[ADC_TABLE_SIZE - 1];
}

float Rads2Rpm(float rads) {
    return rads * 60.0f / (2.0f * 3.141592653589793f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1) {
    if (htim1->Instance == TIM2) {

    	float angleStep = freq / 100;
        float angleStep_tacho = 300 / 100;
        float angleStep_vt = 300 / 100;
        angl1 += angleStep;
        angl2 += angleStep;
        angl3 += angleStep_tacho;
        angl4 += angleStep_vt;

        counter2 +=1;
        if (counter2 == handler_scaler_for_adc) {
            if (adc2_phase == 0) {
                if (HAL_ADC_Start(&hadc2) == HAL_OK)
                    adc2_phase = 1;
            } else {

            	adc2_raw_value = HAL_ADC_GetValue(&hadc2);
            	derivative_buffer_index+=1;
            	if (derivative_buffer_index==DERIVATIVE_WINDOW_SIZE){
            		derivative_buffer_index=0;
            	}
                HAL_ADC_Stop(&hadc2);
                adc2_phase = 0;

                //Определение угла
                derivative = (int)adc2_raw_value - (int)prev_adc_value;
                derivative_sum = derivative_sum + derivative - derivative_buffer[derivative_buffer_index];
                derivative_buffer[derivative_buffer_index]=derivative;
                derivative=(int)roundf(((float)derivative_sum)/(float)DERIVATIVE_WINDOW_SIZE);

                int new_sign = derivative_sign;
                if (derivative > 7) new_sign = 1;
                else if (derivative < 7) new_sign = -1;

                if (derivative_sign != new_sign) {
                    uint32_t current_time = HAL_GetTick();
                    uint32_t time_since_last_extr = current_time - last_extremum_time;
                    if (time_since_last_extr > 5) {
                        // Сохраняем НЕфильтрованное значение
                        float raw_speed = M_PI*1000.0f / (time_since_last_extr*2);

                        // МЕДИАННЫЙ ФИЛЬТР
                        speed_buffer[buffer_index] = raw_speed;

                        if (samples_count < MEDIAN_WINDOW_SIZE) {
                            sorted_indices[samples_count] = buffer_index;
                            position_in_sorted[buffer_index] = samples_count;

                            int i = samples_count;
                            while (i > 0 && speed_buffer[sorted_indices[i-1]] > raw_speed) {
                                sorted_indices[i] = sorted_indices[i-1];
                                position_in_sorted[sorted_indices[i]] = i;
                                i--;
                            }
                            sorted_indices[i] = buffer_index;
                            position_in_sorted[buffer_index] = i;

                            samples_count++;
                        } else {
                            int old_pos = position_in_sorted[buffer_index];

                            for (int i = old_pos; i < MEDIAN_WINDOW_SIZE - 1; i++) {
                                sorted_indices[i] = sorted_indices[i + 1];
                                position_in_sorted[sorted_indices[i]] = i;
                            }

                            int insert_pos = MEDIAN_WINDOW_SIZE - 1;
                            for (int i = 0; i < MEDIAN_WINDOW_SIZE - 1; i++) {
                                if (raw_speed < speed_buffer[sorted_indices[i]]) {
                                    insert_pos = i;
                                    break;
                                }
                            }

                            for (int i = MEDIAN_WINDOW_SIZE - 1; i > insert_pos; i--) {
                                sorted_indices[i] = sorted_indices[i - 1];
                                position_in_sorted[sorted_indices[i]] = i;
                            }

                            sorted_indices[insert_pos] = buffer_index;
                            position_in_sorted[buffer_index] = insert_pos;
                        }

                        int median_index = (samples_count < MEDIAN_WINDOW_SIZE) ? samples_count / 2 : MEDIAN_WINDOW_SIZE / 2;
                        // Присваиваем ОТФИЛЬТРОВАННОЕ значение
                        angular_speed_rad_per_sec = speed_buffer[sorted_indices[median_index]];

                        buffer_index++;
                        if (buffer_index >= MEDIAN_WINDOW_SIZE) {
                            buffer_index = 0;
                        }

                        shaft_angle_deg = (shaft_angle_deg + 90);
                        if (shaft_angle_deg%360==0){
                        	int oborot = shaft_angle_deg/360;
                        }
						if (shaft_angle_deg >= angle_desired){
							stop_flag = 1;
							angular_speed_rad_per_sec = 0;
                        }

                    }
                    last_extremum_time = current_time;
                }
                derivative_sign = new_sign;
                prev_adc_value = adc2_raw_value;

                float angle_error = angle_desired - shaft_angle_deg;
                float dt = (float)handler_scaler_for_adc / 32000.0;

                // PID для угла с антивиндапом
                pid_integral += angle_error * dt;
//                // Антивиндап: ограничиваем интегральную составляющую
//                if (pid_integral > 100.0f) pid_integral = 100.0f;
//                else if (pid_integral < -100.0f) pid_integral = -100.0f;

                float pid_output_angle = KP * angle_error + KI * pid_integral + KD * (angle_error - pid_prev_error) / dt;
                pid_prev_error = angle_error;
                if (pid_output_angle > 110) pid_output_angle = 110;
                else if (pid_output_angle < 20) pid_output_angle = 20;

                // PID для скорости с антивиндапом
                float speed_error = pid_output_angle - angular_speed_rad_per_sec;
                pid_integral_speed += speed_error * dt;

//                // Антивиндап: ограничиваем интегральную составляющую для скорости
//                if (pid_integral_speed > 100.0f) pid_integral_speed = 100.0f;
//                else if (pid_integral_speed < -20.0f) pid_integral_speed = -20.0f;

                pid_output_speed = KP2 * speed_error + KI2 * pid_integral_speed + KD2 * (speed_error - prev_speed_error) / dt;
                prev_speed_error = speed_error;
                if (pid_output_speed > 2) pid_output_speed = 5;
                else if (pid_output_speed < -2) pid_output_speed = -5;
                freq = freq + pid_output_speed;
                if (freq>100) freq = 100;
                if (freq<30) freq = 30;

				sprintf(buffer, "%4d", (int)roundf(shaft_angle_deg));
				SWO_Print(buffer);
				sprintf(buffer, "%4d", (int)roundf(angular_speed_rad_per_sec));
								SWO_Print(buffer);
            }
            counter2 = 0;
        }

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int k;

  for(k=0; k < 360; k++) {
  			sinuses[k] = ((double)sinf(k * PI / 180.0f));
  }

  while (1)
  {

	  // Форматируем строку
	  float sinValue1 = sinuses[(int)roundf(angl1)];
	  float sinValue2 = sinuses[(int)roundf(angl2)];
	  float sinValue3 = sinuses[(int)roundf(angl3)];
	  float sinValue4 = sinuses[(int)roundf(angl3)];

	  if (stop_flag){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  else{
		  //PE9 - PC4-PC5
		  if (sinValue1 >= 0) {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
		  } else {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		  }


		  //PA9 - PA4-PA5
		  if (sinValue2 >= 0) {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  } else {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  }

		  //PE13 - PE10-PE14 (Taho Generator)

		  if (sinValue3 >= 0) {
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
		  } else {
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		  }

		  //PA11 -pa10-pa12 (Vrash transformator)
		  if (sinValue4 >= 0) {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		  } else {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		  }
	  }

	  uint16_t pwmValue1 = (uint16_t)(fabs(sinValue1)*PWM_PERIOD);
	  uint16_t pwmValue2 = (uint16_t)(fabs(sinValue2)*PWM_PERIOD);
	  uint16_t pwmValue3 = (uint16_t)(fabs(sinValue3)*PWM_PERIOD);
	  uint16_t pwmValue4 = (uint16_t)(fabs(sinValue3)*PWM_PERIOD);



	  TIM1->CCR1 = pwmValue1;
	  TIM1->CCR2 = pwmValue2;
	  TIM1->CCR3 = pwmValue3;
	  TIM1->CCR4 = pwmValue4;


	  if (angl1 >= SINE_STEPS){
		  angl1 = (int)angl1 % SINE_STEPS;
		  angl2 = angl1+90;

	  }
	  if (angl2 >= SINE_STEPS)
		  angl2 = (int)angl2 % SINE_STEPS;

	  if (angl3 >= SINE_STEPS)
		  angl3 = (int)angl3 % SINE_STEPS;

	  if (angl4 >= SINE_STEPS)
	  		  angl4 = (int)angl4 % SINE_STEPS;

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT2_Pin */
  GPIO_InitStruct.Pin = BUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA10 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)==GPIO_PIN_SET)
	{
		//angle++;
	}
	else
	{
//		angle--;
	}
}
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
