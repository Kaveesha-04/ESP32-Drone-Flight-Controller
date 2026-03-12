/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usbd_cdc_if.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t MPU6500_ADDR = 0x68 << 1;
uint16_t QMC5883P_ADDR = 0x2C << 1;

uint8_t mpu_raw[14];
uint8_t mag_raw[6];

int16_t ax, ay, az, gx, gy, gz;
int16_t mx, my, mz;

// Floating point physical units
float ax_f, ay_f, az_f;
float gx_rad, gy_rad, gz_rad;
float mx_f, my_f, mz_f;

// Madgwick tuning and state
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.2f;

char msg[150];

uint32_t previousTime = 0;
float roll, pitch, yaw;

// Initial hardcoded offsets - replace with your best known values
float mag_offset_x = 0.0f;
float mag_offset_y = 0.0f;
float mag_offset_z = 0.0f;

// Pulse widths in ticks (1000 = 1ms, 2000 = 2ms)
uint32_t motor1 = 1000;
uint32_t motor2 = 1000;
uint32_t motor3 = 1000;
uint32_t motor4 = 1000;

// Safety flag
uint8_t motors_armed = 0;

uint32_t loopTimer = 0;
const uint32_t loopInterval = 2500; // 2500 microseconds = 400Hz

// PID Gains (Tuning required)
float Kp_roll = 1.2f, Ki_roll = 0.01f, Kd_roll = 0.5f;
float Kp_pitch = 1.2f, Ki_pitch = 0.01f, Kd_pitch = 0.5f;
float Kp_yaw = 2.0f, Ki_yaw = 0.01f, Kd_yaw = 0.0f;

// Errors and Integrals
float error_roll, error_pitch, error_yaw;
float i_term_roll, i_term_pitch, i_term_yaw;
float last_error_roll, last_error_pitch, last_error_yaw;

// Setpoints (Desired angles - 0.0 means level hover)
float setpoint_roll = 0.0f;
float setpoint_pitch = 0.0f;
float setpoint_yaw = 0.0f;

// Throttle management
uint32_t base_throttle = 1300; // Hover base
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
		float az, float mx, float my, float mz, float dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_USB_DEVICE_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	// Start the PWM channels
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	// Arming Sequence: Send 1.0ms (Low) for 3 seconds
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);

	sprintf(msg, "ARMING ESCs... Keep Clear!\r\n");
	CDC_Transmit_FS((uint8_t*) msg, strlen(msg));
	HAL_Delay(3000);

	motors_armed = 1;

	uint8_t tmp;

	// --- STEP 1: RESET & WAKE MPU6500 ---
	tmp = 0x80; // Reset device
	HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x6B, 1, &tmp, 1, 100);
	HAL_Delay(100);

	tmp = 0x01; // Wake up & use Auto Clock
	HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x6B, 1, &tmp, 1, 100);
	HAL_Delay(10);

	// --- STEP 2: ENABLE I2C BYPASS ---
	// Ensure MPU is NOT an I2C Master on its own auxiliary bus
	tmp = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x6A, 1, &tmp, 1, 100);

	// Enable Bypass (Pin 37, Bit 1)
	tmp = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, 0x37, 1, &tmp, 1, 100);
	HAL_Delay(50); // Let the bus stabilize

	// --- STEP 3: INITIALIZE QMC5883P ---
	// Config Reg A: 8-average, 15Hz default, Normal measurement
	tmp = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883P_ADDR, 0x0A, 1, &tmp, 1, 100);

	// Config Reg B: Gain = 1.3 Ga (Standard)
	tmp = 0x20;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883P_ADDR, 0x01, 1, &tmp, 1, 100);

	// Mode Reg: Continuous Measurement Mode
	tmp = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883P_ADDR, 0x02, 1, &tmp, 1, 100);
	HAL_Delay(10);

	// --- I2C SCANNER ---
	char scan_msg[50];
	sprintf(scan_msg, "Scanning I2C Bus...\r\n");
	CDC_Transmit_FS((uint8_t*) scan_msg, strlen(scan_msg));
	HAL_Delay(1000); // Give USB time to connect

	for (uint8_t i = 1; i < 128; i++) {
		// Try to talk to address 'i'
		if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 50)
				== HAL_OK) {
			sprintf(scan_msg, "Found device at: 0x%02X\r\n", i);
			CDC_Transmit_FS((uint8_t*) scan_msg, strlen(scan_msg));
			HAL_Delay(50);
		}
	}
	sprintf(scan_msg, "Scan Complete.\r\n");
	CDC_Transmit_FS((uint8_t*) scan_msg, strlen(scan_msg));
	HAL_Delay(2000); // Pause so you can read the output

	uint8_t baro_id = 0;
	char id_msg[60];

	// Try Door 1: 0x76
	if (HAL_I2C_Mem_Read(&hi2c1, (0x76 << 1), 0xD0, 1, &baro_id, 1, 100)
			== HAL_OK) {
		sprintf(id_msg, "Success at 0x76! ID: 0x%02X\r\n", baro_id);
	}
	// Try Door 2: 0x77
	else if (HAL_I2C_Mem_Read(&hi2c1, (0x77 << 1), 0xD0, 1, &baro_id, 1, 100)
			== HAL_OK) {
		sprintf(id_msg, "Success at 0x77! ID: 0x%02X\r\n", baro_id);
	}
	// Nobody answered
	else {
		sprintf(id_msg, "Failed! No barometer at 0x76 or 0x77.\r\n");
	}

	CDC_Transmit_FS((uint8_t*) id_msg, strlen(id_msg));
	HAL_Delay(2000);

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	uint32_t last_cycle = DWT->CYCCNT;
	uint32_t ticks_per_loop = SystemCoreClock / 400; // Cycles for 400Hz
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	previousTime = HAL_GetTick();
	while (1) {
		// Wait until exactly 2.5ms has passed since the last loop
		while ((DWT->CYCCNT - last_cycle) < ticks_per_loop)
			;
		last_cycle = DWT->CYCCNT;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// --- BUTTON TRIGGERED CALIBRATION (K1 / PE3) ---
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) {
			int16_t mx_min = 32767, mx_max = -32768;
			int16_t my_min = 32767, my_max = -32768;
			int16_t mz_min = 32767, mz_max = -32768;

			int cal_len = sprintf(msg,
					">>> CALIBRATING: ROTATE SENSOR IN FIGURE-8 NOW! <<<\r\n");
			CDC_Transmit_FS((uint8_t*) msg, cal_len);

			uint32_t cal_start = HAL_GetTick();
			while ((HAL_GetTick() - cal_start) < 15000) { // 15 Seconds
				uint8_t status = 0;
				HAL_I2C_Mem_Read(&hi2c1, QMC5883P_ADDR, 0x09, 1, &status, 1,
						10);
				if (status & 0x01) {
					if (HAL_I2C_Mem_Read(&hi2c1, QMC5883P_ADDR, 0x01, 1,
							mag_raw, 6, 10) == HAL_OK) {
						int16_t tx = (int16_t) (mag_raw[1] << 8 | mag_raw[0]);
						int16_t ty = (int16_t) (mag_raw[3] << 8 | mag_raw[2]);
						int16_t tz = (int16_t) (mag_raw[5] << 8 | mag_raw[4]);

						if (tx < mx_min)
							mx_min = tx;
						if (tx > mx_max)
							mx_max = tx;
						if (ty < my_min)
							my_min = ty;
						if (ty > my_max)
							my_max = ty;
						if (tz < mz_min)
							mz_min = tz;
						if (tz > mz_max)
							mz_max = tz;
					}
				}
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
				HAL_Delay(10);
			}

			mag_offset_x = (mx_max + mx_min) / 2.0f;
			mag_offset_y = (my_max + my_min) / 2.0f;
			mag_offset_z = (mz_max + mz_min) / 2.0f;

			cal_len = sprintf(msg,
					"DONE! NEW OFFSETS: X:%.1f Y:%.1f Z:%.1f\r\n", mag_offset_x,
					mag_offset_y, mag_offset_z);
			CDC_Transmit_FS((uint8_t*) msg, cal_len);

			previousTime = HAL_GetTick(); // Reset time to avoid dt spike
		} // <--- Button IF statement ends here

		// --- 1. CALCULATE TRUE delta-t ---
		uint32_t currentTime = HAL_GetTick();
		float dt = (currentTime - previousTime) / 1000.0f;
		previousTime = currentTime;
		if (dt <= 0.0f)
			dt = 0.001f;

		// --- 2. READ MPU6500 ---
		if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, 0x3B, 1, mpu_raw, 14, 100)
				== HAL_OK) {
			ax = (int16_t) (mpu_raw[0] << 8 | mpu_raw[1]);
			ay = (int16_t) (mpu_raw[2] << 8 | mpu_raw[3]);
			az = (int16_t) (mpu_raw[4] << 8 | mpu_raw[5]);
			gx = (int16_t) (mpu_raw[8] << 8 | mpu_raw[9]);
			gy = (int16_t) (mpu_raw[10] << 8 | mpu_raw[11]);
			gz = (int16_t) (mpu_raw[12] << 8 | mpu_raw[13]);
		}

		// --- 3. READ QMC5883P ---
		uint8_t status = 0;
		HAL_I2C_Mem_Read(&hi2c1, QMC5883P_ADDR, 0x09, 1, &status, 1, 100);
		if (status & 0x01) {
			if (HAL_I2C_Mem_Read(&hi2c1, QMC5883P_ADDR, 0x01, 1, mag_raw, 6,
					100) == HAL_OK) {
				mx = (int16_t) (mag_raw[1] << 8 | mag_raw[0]);
				my = (int16_t) (mag_raw[3] << 8 | mag_raw[2]);
				mz = (int16_t) (mag_raw[5] << 8 | mag_raw[4]);
			}
		}

		// --- 4. CONVERT TO PHYSICS UNITS ---
		ax_f = (float) ax;
		ay_f = (float) ay;
		az_f = (float) az;
		gx_rad = (float) gx * 0.000133162f;
		gy_rad = (float) gy * 0.000133162f;
		gz_rad = (float) gz * 0.000133162f;

		mx_f = (float) mx - mag_offset_x;
		my_f = (float) my - mag_offset_y;
		mz_f = (float) mz - mag_offset_z;

		// --- 5. RUN FILTER & CONVERT ---
		MadgwickAHRSupdate(gx_rad, gy_rad, gz_rad, ax_f, ay_f, az_f, mx_f, my_f,
				mz_f, dt);

		roll = -atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29578f;
		pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578f;
		yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29578f;

		// --- FIX 1: Shift North to 0 degrees ---
		yaw -= 90.0f;

		// --- FIX 2: Keep Yaw between 0 and 360 (No negative jumps) ---
		if (yaw < 0.0f) {
			yaw += 360.0f;
		}
		if (yaw >= 360.0f) {
			yaw -= 360.0f;
		}
		// --- 6. PRINT ---
		int len = sprintf(msg, "Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\r\n",
				roll, pitch, yaw);
		CDC_Transmit_FS((uint8_t*) msg, len);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

		// --- PID CALCULATION ---

		// 1. Roll PID
		error_roll = roll - setpoint_roll;
		i_term_roll += error_roll * dt;
		float d_roll = (error_roll - last_error_roll) / dt;
		float out_roll = (Kp_roll * error_roll) + (Ki_roll * i_term_roll) + (Kd_roll * d_roll);
		last_error_roll = error_roll;

		// 2. Pitch PID
		error_pitch = pitch - setpoint_pitch;
		i_term_pitch += error_pitch * dt;
		float d_pitch = (error_pitch - last_error_pitch) / dt;
		float out_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * i_term_pitch) + (Kd_pitch * d_pitch);
		last_error_pitch = error_pitch;

		// 3. Yaw PID (Simple version)
		error_yaw = yaw - setpoint_yaw;
		// Handle the 360 degree wrap-around error
		if (error_yaw > 180) error_yaw -= 360;
		if (error_yaw < -180) error_yaw += 360;
		float out_yaw = Kp_yaw * error_yaw;

		// --- MOTOR MIXER ---
		// Assuming "X" configuration
		if (motors_armed) {
		    motor1 = base_throttle + out_pitch - out_roll + out_yaw; // Front Left
		    motor2 = base_throttle + out_pitch + out_roll - out_yaw; // Front Right
		    motor3 = base_throttle - out_pitch - out_roll - out_yaw; // Back Left
		    motor4 = base_throttle - out_pitch + out_roll + out_yaw; // Back Right

		    // Constrain PWM to safe limits (1000 to 2000)
		    if(motor1 < 1050) motor1 = 1050;
		    if(motor1 > 1900) motor1 = 1900;

		    if(motor2 < 1050) motor2 = 1050;
		    if(motor2 > 1900) motor2 = 1900;

		    if(motor3 < 1050) motor3 = 1050;
		    if(motor3 > 1900) motor3 = 1900;

		    if(motor4 < 1050) motor4 = 1050;
		    if(motor4 > 1900) motor4 = 1900;

		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor1);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor2);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor3);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, motor4);
		}

		// EMERGENCY STOP: If K2 (PC5) is pressed, kill the motors instantly
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
			motors_armed = 0;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
			sprintf(msg, "!!! EMERGENCY STOP ACTIVATED !!!\r\n");
			CDC_Transmit_FS((uint8_t*) msg, strlen(msg));

			}

		}
		/* USER CODE END 3 */
	}

	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void) {
		RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

		/** Supply configuration update enable
		 */
		HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

		/** Configure the main internal regulator output voltage
		 */
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

		while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
		}

		/** Initializes the RCC Oscillators according to the specified parameters
		 * in the RCC_OscInitTypeDef structure.
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
				| RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
				| RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
		RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
		RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)
				!= HAL_OK) {
			Error_Handler();
		}
	}

	/**
	 * @brief I2C1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_I2C1_Init(void) {

		/* USER CODE BEGIN I2C1_Init 0 */

		/* USER CODE END I2C1_Init 0 */

		/* USER CODE BEGIN I2C1_Init 1 */

		/* USER CODE END I2C1_Init 1 */
		hi2c1.Instance = I2C1;
		hi2c1.Init.Timing = 0x00300F38;
		hi2c1.Init.OwnAddress1 = 0;
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0;
		hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
			Error_Handler();
		}

		/** Configure Analogue filter
		 */
		if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
				!= HAL_OK) {
			Error_Handler();
		}

		/** Configure Digital filter
		 */
		if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN I2C1_Init 2 */

		/* USER CODE END I2C1_Init 2 */

	}

	/**
	 * @brief TIM2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM2_Init(void) {

		/* USER CODE BEGIN TIM2_Init 0 */

		/* USER CODE END TIM2_Init 0 */

		TIM_MasterConfigTypeDef sMasterConfig = { 0 };
		TIM_OC_InitTypeDef sConfigOC = { 0 };

		/* USER CODE BEGIN TIM2_Init 1 */

		/* USER CODE END TIM2_Init 1 */
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 63;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 19999;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM2_Init 2 */

		/* USER CODE END TIM2_Init 2 */
		HAL_TIM_MspPostInit(&htim2);

	}

	/**
	 * @brief GPIO Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_GPIO_Init(void) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		/* USER CODE BEGIN MX_GPIO_Init_1 */

		/* USER CODE END MX_GPIO_Init_1 */

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

		/*Configure GPIO pin : PE3 */
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		/*Configure GPIO pin : PA1 */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : PC5 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/*AnalogSwitch Config */
		HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1,
		SYSCFG_SWITCH_PA1_CLOSE);

		/* USER CODE BEGIN MX_GPIO_Init_2 */

		/* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */

// Fast inverse square root
	float invSqrt(float x) {
		return 1.0f / sqrtf(x);
	}

// The Madgwick 9DOF Update Function
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
			float az, float mx, float my, float mz, float dt) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0,
				_2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1,
				q1q2, q1q3, q2q2, q2q3, q3q3;

		// Calculate dynamic sample frequency
		float sampleFreq = 1.0f / dt;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalize accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalize magnetometer measurement
			if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
				recipNorm = invSqrt(mx * mx + my * my + mz * mz);
				mx *= recipNorm;
				my *= recipNorm;
				mz *= recipNorm;

				// Auxiliary variables to avoid repeated arithmetic
				_2q0mx = 2.0f * q0 * mx;
				_2q0my = 2.0f * q0 * my;
				_2q0mz = 2.0f * q0 * mz;
				_2q1mx = 2.0f * q1 * mx;
				_2q0 = 2.0f * q0;
				_2q1 = 2.0f * q1;
				_2q2 = 2.0f * q2;
				_2q3 = 2.0f * q3;
				_2q0q2 = 2.0f * q0 * q2;
				_2q2q3 = 2.0f * q2 * q3;
				q0q0 = q0 * q0;
				q0q1 = q0 * q1;
				q0q2 = q0 * q2;
				q0q3 = q0 * q3;
				q1q1 = q1 * q1;
				q1q2 = q1 * q2;
				q1q3 = q1 * q3;
				q2q2 = q2 * q2;
				q2q3 = q2 * q3;
				q3q3 = q3 * q3;

				// Reference direction of Earth's magnetic field
				hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
						+ _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2
						- mx * q3q3;
				hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
						- my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
				_2bx = sqrtf(hx * hx + hy * hy);
				_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
						- mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
				_4bx = 2.0f * _2bx;
				_4bz = 2.0f * _2bz;

				// Gradient decent algorithm corrective step
				s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
						+ _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
						- _2bz * q2
								* (_2bx * (0.5f - q2q2 - q3q3)
										+ _2bz * (q1q3 - q0q2) - mx)
						+ (-_2bx * q3 + _2bz * q1)
								* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3)
										- my)
						+ _2bx * q2
								* (_2bx * (q0q2 + q1q3)
										+ _2bz * (0.5f - q1q1 - q2q2) - mz);
				s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
						+ _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
						- 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
						+ _2bz * q3
								* (_2bx * (0.5f - q2q2 - q3q3)
										+ _2bz * (q1q3 - q0q2) - mx)
						+ (_2bx * q2 + _2bz * q0)
								* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3)
										- my)
						+ (_2bx * q3 - _4bz * q1)
								* (_2bx * (q0q2 + q1q3)
										+ _2bz * (0.5f - q1q1 - q2q2) - mz);
				s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
						+ _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
						- 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
						+ (-_4bx * q2 - _2bz * q0)
								* (_2bx * (0.5f - q2q2 - q3q3)
										+ _2bz * (q1q3 - q0q2) - mx)
						+ (_2bx * q1 + _2bz * q3)
								* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3)
										- my)
						+ (_2bx * q0 - _4bz * q2)
								* (_2bx * (q0q2 + q1q3)
										+ _2bz * (0.5f - q1q1 - q2q2) - mz);
				s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
						+ _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
						+ (-_4bx * q3 + _2bz * q1)
								* (_2bx * (0.5f - q2q2 - q3q3)
										+ _2bz * (q1q3 - q0q2) - mx)
						+ (-_2bx * q0 + _2bz * q2)
								* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3)
										- my)
						+ _2bx * q1
								* (_2bx * (q0q2 + q1q3)
										+ _2bz * (0.5f - q1q1 - q2q2) - mz);

				recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
				s0 *= recipNorm;
				s1 *= recipNorm;
				s2 *= recipNorm;
				s3 *= recipNorm;

				// Apply feedback step
				qDot1 -= beta * s0;
				qDot2 -= beta * s1;
				qDot3 -= beta * s2;
				qDot4 -= beta * s3;
			}

			// Integrate rate of change of quaternion to yield quaternion
			q0 += qDot1 * (1.0f / sampleFreq);
			q1 += qDot2 * (1.0f / sampleFreq);
			q2 += qDot3 * (1.0f / sampleFreq);
			q3 += qDot4 * (1.0f / sampleFreq);

			// Normalize quaternion
			recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			q0 *= recipNorm;
			q1 *= recipNorm;
			q2 *= recipNorm;
			q3 *= recipNorm;
		}
	}
	/* USER CODE END 4 */

	/* MPU Configuration */

	void MPU_Config(void) {
		MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

		/* Disables the MPU */
		HAL_MPU_Disable();

		/** Initializes and configures the Region and the memory to be protected
		 */
		MPU_InitStruct.Enable = MPU_REGION_ENABLE;
		MPU_InitStruct.Number = MPU_REGION_NUMBER0;
		MPU_InitStruct.BaseAddress = 0x0;
		MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
		MPU_InitStruct.SubRegionDisable = 0x87;
		MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
		MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
		MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
		MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
		MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
		MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

		HAL_MPU_ConfigRegion(&MPU_InitStruct);
		/* Enables the MPU */
		HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

	}

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @retval None
	 */
	void Error_Handler(void) {
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
		}
		/* USER CODE END Error_Handler_Debug */
	}
#ifdef USE_FULL_ASSERT
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
