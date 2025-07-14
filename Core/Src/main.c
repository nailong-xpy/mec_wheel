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
#include "dma.h"
#include "lptim.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "filter.h"
#include "control.h"
#include "driver.h"
#include "motion/motion_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float x;
  float y;
  float z;
} Speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Car_H  18.525
#define  Car_W  11.02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern PID_LocTypeDef Motor_A_PID;
extern PID_LocTypeDef Motor_B_PID;
extern PID_LocTypeDef Motor_C_PID;
extern PID_LocTypeDef Motor_D_PID;

char speed_rx_buffer[4096]__attribute__((section(".out")));
char imu_rx_buffer[4096]__attribute__((section(".out")));

char speed_data_buffer[4096];
char imu_data_buffer[4096];


const float ENCODER_RESOLUTION_AB = 500.0f * 4.0f*30.f;
const float ENCODER_RESOLUTION_CD = 13.0f * 4.0f*30.f;
const float ENCODER_RESOLUTION_XY = 4.0f * 1024.0f;
const float METERS_PER_PULSE_AB = (wheel * 3.1415926535f) / ENCODER_RESOLUTION_AB;//单位cm
const float METERS_PER_PULSE_CD = (wheel * 3.1415926535f) / ENCODER_RESOLUTION_CD;//单位cm
const float METERS_PER_PULSE_XY = (wheel_encoder * 3.1415926535f) / ENCODER_RESOLUTION_XY;//单位cm
Speed_Data Speed_Data_A,Speed_Data_B,Speed_Data_C,Speed_Data_D;

// 新加的正交编码器
Speed_Data Speed_X, Speed_Y;

Speed Speed_car, Position_car, Imu_Speed;

KalmanFilter speed_x, speed_y, acc_x, acc_y, angle_z;

float Motor_A_Set,Motor_B_Set,Motor_C_Set,Motor_D_Set;

float z_target = 0.0f;

// jy62
sensor_data_fifo_s accel_data, gyro_data, angle_data;
float accx,accy;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 整车移动量转换为单轮速度  x:�?????????????+�?????????????-  y:�?????????????+�?????????????-  z:�?????????????+�?????????????-
void Move_Transform(double Vx,double Vy,double Vz)
{
  Motor_C_Set=Vx+Vy-Vz*(Car_H/2+Car_W/2);
  Motor_A_Set=Vx-Vy-Vz*(Car_H/2+Car_W/2);
  Motor_B_Set=Vx+Vy+Vz*(Car_H/2+Car_W/2);
  Motor_D_Set=Vx-Vy+Vz*(Car_H/2+Car_W/2);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_LPTIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xFFFF);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim6);

  // fusion_init();
  // extern icm42688_st icm42688_data;
  // icm42688_init();


  //imu init
  angle_data.dt = 0.01f;
  angle_data.scale = 1.0f;
  angle_data.samples = 0;
  gyro_data.dt = 0.01f;
  gyro_data.scale = 1.0f;
  gyro_data.samples = 0;
  accel_data.dt = 0.01f;
  accel_data.scale = 1.0f;
  accel_data.samples = 0;

  angle_data.scale = 1.0 / 32768.0f * 180.0f;//degree
  gyro_data.scale = 1.0 / 32768.0f * 2000.0f;//dps
  accel_data.scale = 1.0 / 32768.0f * 16.0f * 9.8f * 100.0f;//cm/s^2

  memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));

  char z_cali[3] = {0xff,0xaa,0x52};
  // z归零
  HAL_UART_Transmit(&huart3, (uint8_t *)z_cali, sizeof(z_cali), HAL_MAX_DELAY);
  z_cali[2] = 0x67;
  // acc校准
  HAL_UART_Transmit(&huart3, (uint8_t *)z_cali, sizeof(z_cali), HAL_MAX_DELAY);

  //

  Position_car.x = 0;
  Position_car.y = 0;
  Position_car.z = 0;

  Imu_Speed.x = 0;
  Imu_Speed.y = 0;

  kalman_filter_init(&speed_x, 0.01, 0.2);
  kalman_filter_init(&speed_y, 0.01, 0.2);

  kalman_filter_init(&acc_x, 0.01, 0.5);
  kalman_filter_init(&acc_y, 0.01, 0.5);
  kalman_filter_init(&angle_z, 0.01, 0.5);


  // HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx, sizeof(char));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,imu_rx_buffer,2048);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,speed_rx_buffer,2048);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SetMotor(MOTOR_A, 0);
  SetMotor(MOTOR_B, 0);
  SetMotor(MOTOR_C, 0);
  SetMotor(MOTOR_D, 0);

  // Move_Transform(0,0,PI/2);
  // Move_Transform(5,0,0);
  // Move_Transform(10,10*sqrt(3),0);

  while (1)
  {
    // icm42688_get_gyro();
    // icm42688_get_acc();
    char msg[30];
    // sprintf(msg, "%d,%d,%d,%d,%d,%d\r\n", (int)(Speed_Data_A.speed), (int)(Speed_Data_B.speed), (int)(Speed_Data_C.speed), (int)(Speed_Data_D.speed)
    //   , (int)(Motor_A_PID.er*10), (int)Motor_A_PID.LocSum);

    // sprintf(msg,"%d,%d,%d,%d,%d,%d\r\n", icm42688_data.acc_x, icm42688_data.acc_y, icm42688_data.acc_z,
    //         icm42688_data.gyro_x, icm42688_data.gyro_y, icm42688_data.gyro_z);



    // sprintf(msg, "%d,%d %d\r\n", (int)(Position_car.x*100), (int)(Position_car.y*100), (int)((float)angle_data.z[0] * angle_data.scale * 10));
    // sprintf(msg, "%d,%d,%d,%d \r\n", (int)(Speed_car.x*100), (int)(Speed_car.y*100), (int)(Speed_car.z*100),(int)(Position_car.y*100));

    // sprintf(msg, "%d %d %d %d %d \r\n", (int)(Position_car.x*100), (int)(Position_car.y*100), (int)(Position_car.z*100), (int)(Imu_Speed.x * 100), (int)(gyro_data.z[0] * gyro_data.scale * 10));
    // sprintf(msg, "%d %d %d \r\n", (int)(Speed_car.x*100), (int)(accx * 100), (int)(accy*100));

    sprintf(msg, "%d,%d \r\n", (int)(Speed_X.speed*100), (int)(Speed_Y.speed*100));
    // sprintf(msg, "%d,%d \r\n", TIM8->CNT, LPTIM1->CNT);


    // sprintf(msg, "%d \r\n", Speed_Data_1.current_count);
    HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
    HAL_Delay(20);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void update_speed(Speed_Data* speed, int tim, float mpp) {


  if (tim == 1) {
      speed->current_count = TIM3->CNT;
  }else if (tim == 2) {
      speed->current_count = TIM2->CNT;
  }else if (tim == 3) {
      speed->current_count = TIM5->CNT;
  }else if (tim == 4) {
      speed->current_count = TIM4->CNT;
  }else if (tim == 5) {
      speed->current_count = TIM8->CNT;
  }else if (tim == 8) {
      speed->current_count = LPTIM1->CNT;
  }


  // speed->current_count = tim.CNT;

  speed->delta_pulses = (speed->current_count - speed->encoder_count_prev);
  speed->encoder_count_prev = speed->current_count;
  //防止计数器超�??????????????????65535
  if(speed->delta_pulses>60000)speed->delta_pulses-=65535;
  if(speed->delta_pulses<-60000)speed->delta_pulses+=65535;
  //计算距离
  speed->delta_distance =  speed->delta_pulses* mpp;
  float dt =0.01;//
  //计算车鿿
  speed->speed = speed->delta_distance / dt;
  speed->distance += speed->delta_distance ;
}


void Speed_PID() {
  // PID_output(Motor_A_Set, Speed_Data_A.speed, &Motor_A_PID, 5000);
  // PID_output(Motor_B_Set, Speed_Data_B.speed, &Motor_B_PID, 5000);
  // PID_output(Motor_C_Set, Speed_Data_C.speed, &Motor_C_PID, 5000);
  // PID_output(Motor_D_Set, Speed_Data_D.speed, &Motor_D_PID, 5000);


  PID_output_2(Motor_A_Set, Speed_Data_A.speed, &Motor_A_PID, 5000);
  PID_output_2(Motor_B_Set, Speed_Data_B.speed, &Motor_B_PID, 5000);
  PID_output_2(Motor_C_Set, Speed_Data_C.speed, &Motor_C_PID, 5000);
  PID_output_2(Motor_D_Set, Speed_Data_D.speed, &Motor_D_PID, 5000);

  SetMotor(MOTOR_A, (int)Motor_A_PID.output);
  SetMotor(MOTOR_B, (int)Motor_B_PID.output);
  SetMotor(MOTOR_C, (int)Motor_C_PID.output);
  SetMotor(MOTOR_D, (int)Motor_D_PID.output);
}

void Position_Transform(double Va,double Vb,double Vc,double Vd)
{
  float tmp_vx = (Va + Vb + Vc + Vd) / 4.0d;
  float tmp_vy = (-Va + Vb + Vc - Vd) / 4.0d;
  Speed_car.x = kalman_filter_update(&speed_x, tmp_vx);
  Speed_car.y = kalman_filter_update(&speed_y, tmp_vy);
  Speed_car.z = 1.0f/4.0f*(-Va+Vb-Vc+Vd)/(Car_H+Car_W);
}

void Speed_Int(float dt) {
  Position_Transform(Speed_Data_A.speed, Speed_Data_B.speed, Speed_Data_C.speed, Speed_Data_D.speed);

  float ratio = 0.6;
  float tmp_x = Speed_car.x * ratio + (Imu_Speed.x - 0.1) * (1-ratio);
  float tmp_y = Speed_car.y * ratio + (Imu_Speed.y + 0.115) * (1-ratio);
  Speed_car.x = tmp_x;
  Speed_car.y = tmp_y;
  Imu_Speed.x = tmp_x;
  Imu_Speed.y = tmp_y;

  // Position_car.z += Speed_car.z*dt;
  float tmp_angle_z = kalman_filter_update(&angle_z, angle_data.z[0] * angle_data.scale * 3.1415926535f / 180.0f);
  Position_car.z = tmp_angle_z; // 角度转弧度
  Position_car.x += (Speed_car.x*cosf(Position_car.z)-Speed_car.y*sinf(Position_car.z))*dt;
  Position_car.y += (Speed_car.x*sinf(Position_car.z)+Speed_car.y*cosf(Position_car.z))*dt;
}


void Imu_Speed_Int(float dt) {
  float speed_x = kalman_filter_update(&acc_x, accel_data.y[0] * accel_data.scale);
  float speed_y = kalman_filter_update(&acc_y, accel_data.x[0] * accel_data.scale);

  speed_x += 10.2;
  speed_y += -11.6;

  accy = speed_y;
  accx = speed_x;
  Imu_Speed.x += speed_x * dt;
  Imu_Speed.y += speed_y * dt;
}



int string2int(int start, int end) {
  int result = 0;
  if (speed_data_buffer[start] == '-') {
    start++;
    while (start < end && speed_data_buffer[start] == '0') start++;
    return -string2int(start, end);
  }
  for (int i = start; i <= end; i++) {
    if (speed_data_buffer[i] >= '0' && speed_data_buffer[i] <= '9') {
      result = result * 10 + (speed_data_buffer[i] - '0');
    }
  }
  return result;
}

double motor_speed_x, motor_speed_y, motor_speed_z;
void SpeedHandler() {
    // HAL_UART_Transmit(&huart1, (uint8_t *)data_buffer, strlen(data_buffer), HAL_MAX_DELAY);
  int idx0 = 0;
  while (speed_data_buffer[idx0] != ':') idx0++;
  int idx1 = idx0 + 1;
  while (speed_data_buffer[idx1] != ',') idx1++;
  int idx2 = idx1 + 1;
  while (speed_data_buffer[idx2] != ',') idx2++;
  int idx3 = idx2 + 1;
  while ((speed_data_buffer[idx3] != '\r')&&(speed_data_buffer[idx3] !='\n')) idx3++;


  motor_speed_y = -string2int(idx0+1, idx1-1) / 17.50;
  motor_speed_x = string2int(idx1+1, idx2-1) / 17.50;
  z_target += string2int(idx2+1, idx3-1) / 1000.0; // 0.01 rad/s
  if (z_target > 2.0f * PI) z_target -= 2.0f * PI;
  if (z_target < 0) z_target += 2.0f * PI;

}


bool verify_imu_data(int start) {
  char tmp = 0;
  for (int i=0; i<10; i++) {
    tmp += imu_data_buffer[start+i];
  }
  return tmp == imu_data_buffer[start+10];
}

int16_t parse_tmp[3] = {0};
void parse_imu_data(sensor_data_fifo_s* data, int start) {

  for (int i=0;i<3;i++) {

    int16_t tmp1 = imu_data_buffer[start+2+i*2];
    int16_t tmp2 = imu_data_buffer[start+3+i*2];
    int16_t tmp3 = tmp2<<8 | tmp1;

    parse_tmp[i] = imu_data_buffer[start+3+i*2] << 8 | imu_data_buffer[start+2+i*2];
  }

  data->x[data->samples] = parse_tmp[0];
  data->y[data->samples] = parse_tmp[1];
  data->z[data->samples] = parse_tmp[2];
  data->samples++;

}

void ImuHandler(uint16_t size) {

  accel_data.samples = 0;
  gyro_data.samples = 0;
  angle_data.samples = 0;

  int max_size = (size > 10)?size-10:size;
  for (int i=0; i<max_size; i++) {
    if ((uint16_t)imu_data_buffer[i] == 85) {
      if (verify_imu_data(i)) {
        if (imu_data_buffer[i+1] == 0x51) {
          //加�?�度
          parse_imu_data(&accel_data, i);
        }
        else if (imu_data_buffer[i+1] == 0x52) {
          //角�?�度
          parse_imu_data(&gyro_data, i);
        }
        else if (imu_data_buffer[i+1] == 0x53) {
          //角度
          parse_imu_data(&angle_data, i);
        }
      }
    }
  }
}



bool data_flag = false;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM6) {

      if (data_flag) SpeedHandler(),data_flag = false;

      update_speed(&Speed_Data_A, 1, METERS_PER_PULSE_AB);
      update_speed(&Speed_Data_B, 2, METERS_PER_PULSE_AB);
      update_speed(&Speed_Data_C, 3, METERS_PER_PULSE_CD);
      update_speed(&Speed_Data_D, 4, METERS_PER_PULSE_CD);

    // 新加的正交编码器
      update_speed(&Speed_X, 5, METERS_PER_PULSE_XY);
      update_speed(&Speed_Y, 6, METERS_PER_PULSE_XY);

      Imu_Speed_Int(0.01f);

      Speed_Int(0.01f);

      Speed_PID();

      float P = -6.0f;
      float D = -7.0;
      // if (Position_car.z > 0.1) motor_speed_z = Position_car.z * P + gyro_data.z[0] * gyro_data.scale * D;

      if (Position_car.z - z_target > PI) Position_car.z -= 2.0f * PI;
      if (Position_car.z - z_target < -PI) Position_car.z += 2.0f * PI;

      if (fabs(Position_car.z - z_target) > 0.06)motor_speed_z = (Position_car.z - z_target) * P + (abs((Position_car.z - z_target))) * (Position_car.z - z_target) * D;
      else motor_speed_z = 0;
      Move_Transform(motor_speed_x, motor_speed_y, motor_speed_z);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    strcpy(speed_data_buffer, speed_rx_buffer);
    data_flag = true;
    memset(speed_rx_buffer, 0, sizeof(speed_rx_buffer));
  }
  if (huart->Instance == USART3) {
    memcpy(imu_data_buffer, imu_rx_buffer, Size);
    ImuHandler(55);
    memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));
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
