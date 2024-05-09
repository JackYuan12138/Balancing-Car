#include "motor_control.h"
#include <math.h>

/*配置区*/
#define Mechanical_balance 1.8 // 机械平衡角度
extern int Dead_Zone = 3100;    //电机死区
extern int control_turn = 64;   //转向控制
pid_arg PID = {
	.Balance_Kp = -180,
	.Balance_Kd = -1,
	.Velocity_Kp = 85,
	.Velocity_Ki = 0.25,
	.Turn_Kp = 25,
	.Turn_Kd = 0.05,
};

/**
 * @brief 电机初始化
 */
void Motor_init(void)
{
	/*1-TIM配置*/
	TIM_HandleTypeDef htim;
	htim.Init.Prescaler = 72 - 1;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 1000 - 1;

	/*2-输出比较配置*/
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;//选择PWM模式1
	sConfigOC.Pulse = 0;//CCR的值
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;//输出极性，选择高电平有效

	/*3-GPIO,TIM,TIM主从和输出比较初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP; //复用推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	/* MOTOR_A_START */
	#if MOTOR_A
	GPIO_InitStructure.Pin = MOTOR_APWM_Pin;
	if (MOTOR_APWM_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_APWM_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	HAL_GPIO_Init(MOTOR_APWM_Port, &GPIO_InitStructure);
	htim.Instance = MOTOR_A_PWM_TIM;
	if (HAL_TIM_PWM_Init(&htim) != HAL_OK) {
		Error_Handler("PWM_Init");
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, MOTOR_A_PWM_TIM_CCR_CHANNEL) != HAL_OK) {
		Error_Handler("TWM_ConfigChannel");
	}
	if (HAL_TIM_PWM_Start(&htim, MOTOR_A_PWM_TIM_CCR_CHANNEL) != HAL_OK) {
		Error_Handler("PWM_Start");
	}
	#endif
	/* MOTOR_A_END */
	/* MOTOR_B_START */
	#if MOTOR_B
	GPIO_InitStructure.Pin = MOTOR_BPWM_Pin;
	if (MOTOR_BPWM_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_BPWM_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	HAL_GPIO_Init(MOTOR_BPWM_Port, &GPIO_InitStructure);
	htim.Instance = MOTOR_B_PWM_TIM;
	if (HAL_TIM_PWM_Init(&htim) != HAL_OK) {
		Error_Handler("PWM_Init");
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, MOTOR_B_PWM_TIM_CCR_CHANNEL) != HAL_OK) {
		Error_Handler("TWM_ConfigChannel");
	}
	if (HAL_TIM_PWM_Start(&htim, MOTOR_B_PWM_TIM_CCR_CHANNEL) != HAL_OK) {
		Error_Handler("PWM_Start");
	}
	#endif
	/* MOTOR_B_END */
}

/**
 * @brief 编码器初始化
 */
void Encoder_Init(void)
{
	TIM_HandleTypeDef htim;
	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	GPIO_InitTypeDef GPIO_InitStructure;

	htim.Init.Prescaler = 0;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 65535;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
	/* MOTOR_A_START */
	#if MOTOR_A
	GPIO_InitStructure.Pin = MOTOR_AENCODER_Pin1 | MOTOR_AENCODER_Pin2;
	if (MOTOR_AENCODER_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_AENCODER_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	HAL_GPIO_Init(MOTOR_AENCODER_Port, &GPIO_InitStructure);
	htim.Instance = MOTOR_A_ENCODER_TIM;
	if (HAL_TIM_Encoder_Init(&htim, &sConfig) != HAL_OK) {
		Error_Handler("MOTOR_A_ENCODER_Init");
	}
	if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK) {
		Error_Handler("MOTOR_A_ENCODER_MasterConfig");
	}
	if(HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler("MOTOR_A_ENCODER_Start");
	}
	#endif
	/* MOTOR_A_END */
	/* MOTOR_B_START */
	#if MOTOR_B
	GPIO_InitStructure.Pin = MOTOR_BENCODER_Pin1 | MOTOR_BENCODER_Pin2;
	if (MOTOR_BENCODER_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (MOTOR_BENCODER_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	HAL_GPIO_Init(MOTOR_BENCODER_Port, &GPIO_InitStructure);
	htim.Instance = MOTOR_B_ENCODER_TIM;
	if (HAL_TIM_Encoder_Init(&htim, &sConfig) != HAL_OK) {
		Error_Handler("MOTOR_B_ENCODER_Init");
	}
	if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK) {
		Error_Handler("MOTOR_B_ENCODER_MasterConfig");
	}
	if(HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler("MOTOR_B_ENCODER_Start");
	}
	#endif
}

/**
 * @brief 设置电机速度
 * @param motorX 电机编号(1：电机A；2：电机B)
 * @param speed 速度值(-100~100)
 * @note PWM的占空比为CCR/ARR，而ARR设计为100，所以CCR的值即为占空比
 */
void Motor_setSpeed(uint8_t motorX, int16_t speed)
{
	switch (motorX) {
		case 1:
			/*调整方向*/
			if (speed == 0) {
				/*制动*/
				HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET);
			} else if (speed < 0) {
				/*反转*/
				speed = -speed;
				HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET);
			} else {
				/*正转*/
				HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET);
			}

			/*设置速度*/
			setCCR(MOTOR_A_PWM_TIM, MOTOR_A_PWM_TIM_CCR_CHANNEL, (uint32_t)Dead_Zone + speed * 1.17);
			break;
		case 2:
			/* MOTOR_B_START */
			if (speed == 0) {
				/*制动*/
				HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET);
			} else if (speed < 0) {
				/*反转*/
				speed = -speed;
				HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET);
			} else {
				/*正转*/
				HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET);
			}

			/*设置速度*/
			setCCR(MOTOR_B_PWM_TIM, MOTOR_B_PWM_TIM_CCR_CHANNEL, (uint32_t)Dead_Zone + speed * 1.17);
			break;
		default:
			break;
	}
}

/**
 * @brief 获取编码器速度
 * @param EncoderX 编码器编号(1：编码器1；2：编码器2)
 */
int16_t Motor_getSpeed(uint8_t EncoderX)
{
	int16_t speed;

	switch (EncoderX) {
		case 1:
			speed = (int16_t)MOTOR_A_ENCODER_TIM->CNT;
			MOTOR_A_ENCODER_TIM->CNT = 0;
			break;
		case 2:
			speed = (int16_t)MOTOR_B_ENCODER_TIM->CNT;
			MOTOR_B_ENCODER_TIM->CNT = 0;
			break;
		default:
			speed = 0;
	}

	return speed;
}

/**
 * @brief 直立环PD控制
 * @param Angle: x轴的角度
 * @param Gyro: x轴的角速度
 * @return 经过PID转换之后的PWM值
 */
uint32_t Motor_verticalRingPD(float Angle, float Gyro)
{
	float Bias;//目标偏差
	uint32_t balance;

	Bias = Angle - Mechanical_balance;//目标值减去机械中值（不一定为0）
	balance = PID.Balance_Kp * Bias + Gyro * PID.Balance_Kd;

	return balance;
}

/**
 * @brief 垂直速度PI控制
 * @param encoder_left: 左编码器值
 * @param encoder_right: 右编码器值
 * @param Angle: 角度
 * @param Movement: 运动
 * @return 速度
 */
float Motor_verticalSpeedPI(int encoder_left, int encoder_right, float Angle, float Movement)
{
	static float Velocity, Encoder_Least, Encoder;
	static float Encoder_Integral;
	Encoder_Least = (encoder_left + encoder_right) - 0; //获取最新速度偏差=测量速度（左右编码器之和）-目标速度（此处为零）
	Encoder *= 0.8f;									//一阶低通滤波器 ，上次的速度占85%
	Encoder += Encoder_Least * 0.2f;                   //一阶低通滤波器， 本次的速度占15%
	Encoder_Integral += Encoder;                       //积分出位移 积分时间：10ms
	Encoder_Integral = Encoder_Integral - Movement;
	//if(Movement == 0 ) Encoder_Integral=0;
	if (Encoder_Integral > 10000)  	Encoder_Integral = 10000;           //积分限幅
	if (Encoder_Integral < -10000)	  Encoder_Integral = -10000;            //积分限幅

	Velocity = Encoder * PID.Velocity_Kp + Encoder_Integral * PID.Velocity_Ki;      //速度控制

	if (Motor_turnOff(Angle) == 1) {
		Encoder_Integral = 0;//电机关闭后清除积分
	}
	return Velocity;
}

/**
 * @brief 垂直转向PD控制
 * @param taget_yaw: 目标偏航角
 * @param yaw: 当前偏航角
 * @param gyro: 当前角速度
 * @return 转向
 */
float Motor_verticalTurnPD(float taget_yaw, float yaw, float gyro)
{
	float Turn, Bias_yaw;

	Bias_yaw = taget_yaw - yaw;
	if (Bias_yaw < -180) Bias_yaw += 360;
	if (Bias_yaw > 180) Bias_yaw -= 360;

	Turn = -Bias_yaw * PID.Turn_Kp - gyro * PID.Turn_Kd;

	return Turn;
}

/**
 * @brief PWM限幅函数
 * @param motor1: 电机1的PWM
 * @param motor2: 电机2的PWM
 */
void Motor_PWMLimiting(int* motor1, int* motor2)
{
	int Amplitude = 7999;
	if (*motor1 < -Amplitude) *motor1 = -Amplitude;
	if (*motor1 > Amplitude)  *motor1 = Amplitude;
	if (*motor2 < -Amplitude) *motor2 = -Amplitude;
	if (*motor2 > Amplitude)  *motor2 = Amplitude;
}

uint8_t FS_state = 0;
/**
 * @brief 关闭电机
 * @param Angle: x轴角度值
 * @return 1:小车当前处于停止状态/0:小车当前处于正常状态
 */
uint8_t Motor_turnOff(const float Angle)
{
	uint8_t temp;
	if (fabs(Angle) > 60) {
		FS_state = 1;
		temp = 1;
		setCCR(MOTOR_A_PWM_TIM, MOTOR_A_PWM_TIM_CCR_CHANNEL, 0);
		setCCR(MOTOR_B_PWM_TIM, MOTOR_B_PWM_TIM_CCR_CHANNEL, 0);
	} else {
		temp = 0;
	}
	FS_state = 0;
	return temp;
}
