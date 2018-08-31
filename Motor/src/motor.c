#include "motor.h"
int counter=0,counter2=0;

TIM_OC_InitTypeDef sConfig;

/*
 * this function initializes the timer for the encoder pins for both motors
*/
void MotionTimInit(TIM_HandleTypeDef *htim){
	uint16_t psc				= MOTION_PSC,	 //freq = 1MHz
			period				= MOTION_PERIOD; //freq = 1kHz -> every 1ms  change period to inc/dec the freq of motion
												 //!!!!!!period max = 65500
	htim->Instance				 = TIM5;
	htim->Init.Period			 = period;
	htim->Init.Prescaler		 = psc;
	htim->Init.ClockDivision	 = TIM_CLOCKDIVISION_DIV1;
	htim->Init.CounterMode		 = TIM_COUNTERMODE_UP;
	htim->Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(htim);
	HAL_TIM_Base_Start_IT(htim);
}

void SerialUploadTimInit(TIM_HandleTypeDef *htim){
	uint16_t psc = SERIALUPLOAD_PSC,				//freq = 100kHz
			period = SERIALUPLOAD_PERIOD;			//freq = 100Hz -> every 10ms  change period to inc/dec the freq of motion
													//!!!!!!period max = 65500
	htim->Instance				 = TIM7;
	htim->Init.Period			 = period;
	htim->Init.Prescaler		 = psc;
	htim->Init.ClockDivision	 = TIM_CLOCKDIVISION_DIV1;
	htim->Init.CounterMode		 = TIM_COUNTERMODE_UP;
	htim->Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(htim);
	HAL_TIM_Base_Start_IT(htim);
	robotState.x = 0;
	robotState.y = 0;
	robotState.dotx = 0;
	robotState.doty = 0;
	robotState.ddoty = 0;
	robotState.ddoty = 0;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################
   TIMx Peripheral clock enable*/
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_TIM7_CLK_ENABLE();

/*  ##-2- Configure the NVIC for TIMx ########################################
   Set Interrupt Group Priority*/
  HAL_NVIC_SetPriority(TIM5_IRQn, 4, 0);
  HAL_NVIC_SetPriority(TIM7_IRQn, 4, 1);

   /*Enable the TIMx global Interrupt*/
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM5){
		moveRobot();
	}
	else if (htim->Instance == TIM7){
//		if(Robot.sendOK)
			SendData();
	}
}

void EncoderInit(TIM_HandleTypeDef *encoderL,TIM_HandleTypeDef *encoderR){

	//Left motor encoder initialization
	encoderL->Instance 					= TIM3;
	encoderL->Init.Period            	= 0xFFFF;
	encoderL->Init.Prescaler         	= 0;
	encoderL->Init.ClockDivision     	= 0;
	encoderL->Init.CounterMode       	= TIM_COUNTERMODE_UP;
	encoderL->Init.RepetitionCounter 	= 0;

	TIM_Encoder_InitTypeDef sEncoderConfig;

	sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
	sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
	sEncoderConfig.IC1Filter          = 3;							///?????

	sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
	sEncoderConfig.IC2Filter          = 3;
	HAL_TIM_Encoder_Init(encoderL, &sEncoderConfig);
	HAL_TIM_Encoder_Start(encoderL, TIM_CHANNEL_ALL);		//TI1&TI2


	//Right motor encoder initialization
	//*
	encoderR->Instance 					= TIM2;
	encoderR->Init.Period            	= 0xFFFF;
	encoderR->Init.Prescaler         	= 0;
	encoderR->Init.ClockDivision     	= 0;
	encoderR->Init.CounterMode       	= TIM_COUNTERMODE_UP;
	//encoderR->Init.RepetitionCounter 	= 3;
	HAL_TIM_Encoder_Init(encoderR, &sEncoderConfig);
	HAL_TIM_Encoder_Start(encoderR, TIM_CHANNEL_ALL);//*/
}

void MT_Input_Init(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR)
{
	/*time base configuration for Left motor */
	motorL->Instance				= TIM4;
	motorL->Init.Period 			= M_PWM_TickPeriod;
	motorL->Init.Prescaler 			= 0;
	motorL->Init.ClockDivision		= 0;
	motorL->Init.CounterMode		= TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(motorL);

	/* time base configuration for Right motor*/
	//gamw to antitheo sou motorL=motorR de doulevei kai de leitourgei o timer
	motorR->Instance				= TIM4;
	motorR->Init.Period 			= M_PWM_TickPeriod;
	motorR->Init.Prescaler 			= 0;
	motorR->Init.ClockDivision		= 0;
	motorR->Init.CounterMode		= TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(motorR);

	TIM_OC_InitTypeDef		hoctim;

	hoctim.OCMode		= TIM_OCMODE_PWM1;
	hoctim.Pulse		= 0;
	hoctim.OCIdleState	= TIM_OCIDLESTATE_SET;
	hoctim.OCNIdleState	= TIM_OCNIDLESTATE_SET;
	hoctim.OCPolarity	= TIM_OCPOLARITY_HIGH;
	hoctim.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	hoctim.OCFastMode	= TIM_OCFAST_ENABLE;
	/*	motorR pin1(MTy1) channel configuration*/
	HAL_TIM_PWM_ConfigChannel(motorL,&hoctim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motorL,TIM_CHANNEL_1);
	/*	motorR pin2(MTy2) channel configuration*/
	HAL_TIM_PWM_ConfigChannel(motorL,&hoctim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(motorL,TIM_CHANNEL_2);
	/*	motorR pin1(MTx1) channel configuration*/
	HAL_TIM_PWM_ConfigChannel(motorR,&hoctim,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(motorR,TIM_CHANNEL_3);
	/*	motorR pin2(MTx2) channel configuration*/
	HAL_TIM_PWM_ConfigChannel(motorR,&hoctim,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(motorR,TIM_CHANNEL_4);
}

void MotorInit(void){
	MT_Input_Init(&motorR,&motorL);
	EncoderInit(&encoderR,&encoderL);
	Robot.volt = 7.5;
	Robot.refPwmL = 0;
	Robot.refPwmR = 0;
	Robot.vR	  = 0;
	Robot.vL	  = 0;
	Robot.wL	  = 0;
	Robot.wR	  = 0;
	Robot.newCom  = false;
	Robot.sendOK  = false;
	Robot.errSumL = 0;
	Robot.errSumR = 0;
	RobotNew.a 	  = 1.0;
	returnData[12] = '\n';
	robotSetPos(0,0,0);
	MotionTimInit(&motionTimer);
	SerialUploadTimInit(&serialUpTimer);
	max_Vel.tang_max = TANG_VEL_MAX;
	max_Vel.ang_max = ANG_VEL_MAX;
	PID.Ki = I;
	PID.Kp = P;
	Stop(&motorL, &motorR);

}

void Stop(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR){
	StopL(motorL);
	StopR(motorR);
}

void StopL(TIM_HandleTypeDef *motorL){
	PWM_MTy1(motorL,0);
	PWM_MTy2(motorL,0);
}

void StopR(TIM_HandleTypeDef *motorR){
	PWM_MTx1(motorR,0);
	PWM_MTx2(motorR,0);
}


void MoveB(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR, int PercentWidth){
//	if (PercentWidth>50)
//		PercentWidth = 50;
	PWM_MTy1(motorL,PercentWidth);
	PWM_MTx2(motorR,PercentWidth);
}

void MoveF(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR, int PercentWidth){
//	if (PercentWidth>50)
//			PercentWidth = 50;
	PWM_MTy2(motorL,PercentWidth);
	PWM_MTx1(motorR,PercentWidth);
}

void MoveRF(TIM_HandleTypeDef *motor){
	PWM_MTx1(motor,Robot.pwmR);
}

void MoveLF(TIM_HandleTypeDef *motor){
	PWM_MTy2(motor,Robot.pwmL);
}

void MoveRB(TIM_HandleTypeDef *motor){
	PWM_MTx2(motor,Robot.pwmR);
}

void MoveLB(TIM_HandleTypeDef *motor){
	PWM_MTy1(motor,Robot.pwmL);
}
/*change pulse width for MTy1 pin PB6/TIM4_CH1*/
void PWM_MTy1_per(TIM_HandleTypeDef *htim, int PercentWidth)
{
	int width = (PercentWidth/100.0)*htim->Init.Period;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, width);
}
/*change pulse width for MTy2 pin PB7/TIM4_CH2*/
void PWM_MTy2_per(TIM_HandleTypeDef *htim, int PercentWidth)
{
	int width = (PercentWidth/100.0)*htim->Init.Period;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, width);
}
/*change pulse width for MTx1 pin PB8/TIM4_CH3*/
void PWM_MTx1_per(TIM_HandleTypeDef *htim, int PercentWidth)
{
	int width = (PercentWidth/100.0)*htim->Init.Period;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, width);
}
/*change pulse width for MTx2 pin PB9/TIM4_CH4*/
void PWM_MTx2_per(TIM_HandleTypeDef *htim, int PercentWidth)
{
	int width = (PercentWidth/100.0)*htim->Init.Period;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, width);
}

/*change pulse width for MTy1 pin PB6/TIM4_CH1*/
void PWM_MTy1(TIM_HandleTypeDef *htim, float pwm)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pwm*htim->Init.Period);
}
/*change pulse width for MTy2 pin PB7/TIM4_CH2*/
void PWM_MTy2(TIM_HandleTypeDef *htim, float pwm)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pwm*htim->Init.Period);
}
/*change pulse width for MTx1 pin PB8/TIM4_CH3*/
void PWM_MTx1(TIM_HandleTypeDef *htim, float pwm)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, pwm*htim->Init.Period);
}
/*change pulse width for MTx2 pin PB9/TIM4_CH4*/
void PWM_MTx2(TIM_HandleTypeDef *htim, float pwm)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, pwm*htim->Init.Period);
}

//Msp init funcs
/*
 * This function initializes pin and enables clocks for output PWM pins. These pins
 * are the input pins for motors MTy1, MTy2, MTx1, MTx2
*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	/*Left & Right Motors input pins in TIM4*/
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin			= GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; //Motor Left -> PB6 & PB7, Motor Right -> PB8 & PB9
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  __HAL_RCC_TIM2_CLK_ENABLE();	//Motor Left encoder timer
  __HAL_RCC_TIM3_CLK_ENABLE();	//Motor Right encoder timer

  /* Enable GPIO Channels Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();	//ENCyA
  __HAL_RCC_GPIOB_CLK_ENABLE();	//ENCxA / ENCxB / ECyB

  /*//////////////////MOTOR 1 ////////////////////////////////////////////////*/
  /*##-2- Configure I/Os #####################################################*/
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull 		= GPIO_PULLUP;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

  /*Left Motor encoder encyA(PA15,TIM2,CH1) configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Left Motor encoder encxA (PB3,TIM2,CH2) configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Right Motor encoder ENCxB (PB4, TIM3,CH1) & encyB (PB5, TIM3,CH2) configuration */
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void EncReset(TIM_HandleTypeDef *motor){
	__HAL_TIM_SET_COUNTER(motor,0);
}
///P controller
void Pcontrol(void){
	volatile float pwmL,pwmR,errorL,errorR,outputR,outputL;
	pwmL = vel2pwm(Robot.wL)*10;//	pwmL = vel2pwm(Robot.wL)*10;
	pwmR = vel2pwm(Robot.wR)*10;
	errorL = Robot.refPwmL - pwmL;
	errorR = Robot.refPwmR - pwmR;
	Robot.errSumL = pwmL;//+= (errorL * UPDTIME);
	Robot.errSumR = pwmR;//+= (errorR * UPDTIME);
	outputR = errorR*PID.Kp + Robot.errSumR*PID.Ki;//Robot.pwmR + errorR*P + Robot.errSumR*I;
	outputL = errorL*PID.Kp + Robot.errSumL*PID.Ki;//Robot.pwmL + errorL*P + Robot.errSumL*I;
	/*
	if(outputR>50){
		outputR = 50;
	}
	if(outputL>50){
		outputL = 50;
	}//*/
	if (outputR>0)
		Robot.pwmR  = outputR;
	else
		Robot.pwmR = 0;

	if (outputL>0)
		Robot.pwmL = outputL;
	else
		Robot.pwmL = 0;
}

void PIDcontrolNew(void){
	volatile float pwmL,pwmR,errorL,errorR,outputR,outputL;
	pwmL = vel2pwm(RobotNew.wL);//	pwmL = vel2pwm(Robot.wL)*10;
	pwmR = vel2pwm(RobotNew.wR);
	errorL = Robot.refPwmL - pwmL;
	errorR = Robot.refPwmR - pwmR;
	RobotNew.errSumL += (errorL * UPDTIME);
	RobotNew.errSumR += (errorR * UPDTIME);
	outputR = errorR*PID.Kp + RobotNew.errSumR*PID.Ki;//Robot.pwmR + errorR*P + Robot.errSumR*I;
	outputL = errorL*PID.Kp + RobotNew.errSumL*PID.Ki;//Robot.pwmL + errorL*P + Robot.errSumL*I;
	if (outputR>0)
		Robot.pwmR  = outputR;
	else
		Robot.pwmR = 0;

	if (outputL>0)
		Robot.pwmL = outputL;
	else
		Robot.pwmL = 0;
}

//#################	Motion Control	################//
void updateRobot(void){
	//update previous tick
	Robot.prevTickL = Robot.tickL;
	Robot.prevTickR = Robot.tickR;
	//update tick
	Robot.tickL		= __HAL_TIM_GET_COUNTER(&encoderL);
	Robot.tickR		= __HAL_TIM_GET_COUNTER(&encoderR);
	//update speed of robot
	//Forward move
	if((Robot.prevTickL>Robot.tickL && Robot.dirL == FORWARD) && 
		(Robot.prevTickR>Robot.tickR && Robot.dirR == FORWARD)){ 	//both encoder counters overflow pote omws
		return;
	}

	else if(Robot.prevTickL>Robot.tickL && Robot.dirL == FORWARD){						//only Left enc overflow
		Robot.wR	= (2*M_PI)/CPM*(Robot.tickR - Robot.prevTickR)/UPDTIME;
		return;
	}
	else if(Robot.prevTickR>Robot.tickR && Robot.dirR == FORWARD){						//only Right enc overflow
		Robot.wL	= (2*M_PI)/CPM*(Robot.tickL - Robot.prevTickL)/UPDTIME;
		return;
	}
	//Backward move
	else if((Robot.prevTickL<Robot.tickL && Robot.dirL == BACKWARD) && 
			(Robot.prevTickR<Robot.tickR && Robot.dirR == BACKWARD)){ //both encoder counters underflow
		return;
	}
	else if(Robot.prevTickL<Robot.tickL && Robot.dirL == BACKWARD){						//only Left enc underflow
		Robot.wR	= (2*M_PI)/CPM*(Robot.prevTickR - Robot.tickR)/UPDTIME;
		return;
	}
	else if(Robot.prevTickR<Robot.tickR && Robot.dirR == BACKWARD){
		Robot.wL	=  (2*M_PI)/CPM*(Robot.prevTickL - Robot.tickL)/UPDTIME;
		return;
	}
	if(Robot.dirR == FORWARD){
		Robot.wR	= (2*M_PI)/CPM*(Robot.tickR - Robot.prevTickR)/UPDTIME;
	}
	else{
		Robot.wR	= (2*M_PI)/CPM*(Robot.prevTickR - Robot.tickR)/UPDTIME;
	}
	if(Robot.dirL == FORWARD){
		Robot.wL	= (2*M_PI)/CPM*(Robot.tickL - Robot.prevTickL)/UPDTIME;
	}
	else{
		Robot.wL	= (2*M_PI)/CPM*(Robot.prevTickL - Robot.tickL)/UPDTIME;
	}
}

void updateRobotNew(void){		//odometry from book
	//update previous tick
	double cm;
	RobotNew.prevTickL = RobotNew.tickL;
	RobotNew.prevTickR = RobotNew.tickR;
	//update tick
	RobotNew.tickL		= __HAL_TIM_GET_COUNTER(&encoderL);
	RobotNew.tickR		= __HAL_TIM_GET_COUNTER(&encoderR);
	cm = (2*M_PI*R)/CPM;
	//update speed of robot
	//Forward move
	if((RobotNew.prevTickL>RobotNew.tickL && RobotNew.dirL == FORWARD) && 
		(RobotNew.prevTickR>RobotNew.tickR && RobotNew.dirR == FORWARD)){ 		//both encoder counters overflow pote omws
		return;
	}

	else if(RobotNew.prevTickL>RobotNew.tickL && RobotNew.dirL == FORWARD){		//only Left enc overflow
		RobotNew.sR	= cm*(RobotNew.tickR - RobotNew.prevTickR);
		return;
	}
	else if(RobotNew.prevTickR>RobotNew.tickR && RobotNew.dirR == FORWARD){		//only Right enc overflow
		RobotNew.sL	= cm*(RobotNew.tickL - RobotNew.prevTickL);
		return;
	}
	//Backward move
	else if((RobotNew.prevTickL<RobotNew.tickL && RobotNew.dirL == BACKWARD) && 
			(RobotNew.prevTickR<RobotNew.tickR && RobotNew.dirR == BACKWARD)){	//both encoder counters underflow
		return;
	}
	else if(RobotNew.prevTickL<RobotNew.tickL && RobotNew.dirL == BACKWARD){	//only Left enc underflow
		RobotNew.sR	= cm*(RobotNew.prevTickR - RobotNew.tickR);
		RobotNew.wR = RobotNew.sR/UPDTIME;
		RobotNew.s  = (RobotNew.sL + RobotNew.sR)/2.0;
		return;
	}
	else if(RobotNew.prevTickR<RobotNew.tickR && RobotNew.dirR == BACKWARD){
		RobotNew.sL	=  cm*(RobotNew.prevTickL - RobotNew.tickL);
		RobotNew.wL = RobotNew.sL/UPDTIME;
		RobotNew.s  = (RobotNew.sL + RobotNew.sR)/2.0;
		return;
	}
	if(RobotNew.dirR == FORWARD){
		RobotNew.sR	= cm*(RobotNew.tickR - RobotNew.prevTickR);
	}
	else{
		RobotNew.sR	= cm*(RobotNew.prevTickR - RobotNew.tickR);
	}
	if(RobotNew.dirL == FORWARD){
		RobotNew.sL	= cm*(RobotNew.tickL - RobotNew.prevTickL);
	}
	else{
		RobotNew.sL	= cm*(RobotNew.prevTickL - RobotNew.tickL);
	}
	RobotNew.s  		= (RobotNew.sL + RobotNew.sR)/2.0;
	RobotNew.wR = RobotNew.sR/UPDTIME;
	RobotNew.wL = RobotNew.sL/UPDTIME;
}
/*
This function returns the actual number of revolution that the wheel has made
The wheel's specs(9.7:1 Metal Gearmotor 25Dx48L mm)
tick is the output of the encoder
*/
float tick2wheelRev(int tick){
	return (float)(tick)/CPM;
}

float wheelRev2tick(float cpr){
	return cpr*CPM;
}
/*
this function returns the distance that the wheel has traveled
pi 	-> 3.14
R	-> wheel's radius
*/
float wheelDist(float cpr){
	return 2*M_PI*R*cpr;
}

float dist2wheelRev(float dist){
	return (dist)/(2*M_PI*R);
}

float pwm2vel(float pwm){
	float	Kt = 0.035,
			Ke = 0.035,
			r  = 2.72,
			b  = 8.21*1e-4;
	return (Kt/(r*b+Ke*Kt))*pwm*Robot.volt/100;
}

float vel2pwm(float w){
	float 	Kt = 0.035,
			Ke = 0.035,
			r  = 2.72,
			b  = 8.21*1e-4;
	return (w/Robot.volt)*(r*b+Ke*Kt)/Kt*100;
}

float angVel2LinVel(float w){
	return w*R;
}

float LinVel2angVel(float v){
	return v/R;
}

/*
 * New function to control robot's movement*******should be called with interrupt to update RobotState
 */
void movePanoramixHandle(volatile double tangVel, volatile double angVel){
	double ur, ul;
	ur = vel2pwm(tangVel + angVel/2.0);
	ul = vel2pwm(tangVel + angVel/2.0);
	ur > 0 ? (Robot.dirR = FORWARD) : (Robot.dirR = BACKWARD);
	ul > 0 ? (Robot.dirL = FORWARD) : (Robot.dirL = BACKWARD);
	Robot.refPwmL = fabs(ul);
	Robot.refPwmR = fabs(ur);
}

void movePanoramixTeleopKeyHandle( double tangVel, double angVel){
	double pwmR, pwmL;

	if(tangVel==0 && angVel==0){
			pwmL = pwmR = 0;
		}
	else if(tangVel!=0){
		pwmR = vel2pwm(LinVel2angVel(tangVel - angVel/RobotNew.a));
		pwmL = vel2pwm(LinVel2angVel(tangVel + angVel/RobotNew.a));
		pwmR > 0 ? (Robot.dirR = FORWARD) : (Robot.dirR = BACKWARD);
		pwmL > 0 ? (Robot.dirL = FORWARD) : (Robot.dirL = BACKWARD);
	}
	else if(angVel!=0){
		pwmR = vel2pwm(LinVel2angVel(tangVel - angVel/RobotNew.a));
		pwmL = vel2pwm(LinVel2angVel(tangVel + angVel/RobotNew.a));
		pwmR > 0 ? (Robot.dirR = FORWARD) : (pwmR = pwmL/RobotNew.a);
		pwmL > 0 ? (Robot.dirL = FORWARD) : (pwmL = pwmR/RobotNew.a);

	}
	Robot.refPwmL = fabs(pwmL);
	Robot.refPwmR = fabs(pwmR);
}

void moveRobot(){
	updateRobot();
	updateRobotNew();
	calcPos();
	calcPosNew();
	PIDcontrolNew();
	if(Robot.newCom){
		Stop(&motorL, &motorR);
		Robot.errSumL = 0;
		Robot.errSumR = 0;
		counter++;
	}
	else{
		if((Robot.dirL == STOP && Robot.dirR == STOP)){	//|| (time > Robot.time)
			Stop(&motorR,&motorL);
		}
		if(Robot.dirR == FORWARD){
			MoveRF(&motorR);
		}
		else if(Robot.dirR == BACKWARD){
			MoveRB(&motorR);
		}
		if(Robot.dirL == FORWARD){
			MoveLF(&motorL);
		}
		else if(Robot.dirL == BACKWARD){
			MoveLB(&motorL);
		}
	}
	if(counter > 2){
		Robot.newCom = false;
		counter = 0;
	}
}

void robotSetPos(float x, float y, float theta){
	Robot.x		= x;
	Robot.y		= y;
	Robot.theta	= theta;
	Robot.thetaOld = Robot.theta;
}

void SendData(void){

	robotState.x = RobotNew.x;			// Robot.refPwmR;
	robotState.y = RobotNew.y;			//Robot.refPwmL;
	robotState.theta = RobotNew.theta;
	robotState.dotx = RobotNew.dotx;	//Robot.dotx; //Robot.pwmR;
	robotState.doty = RobotNew.doty;	//Robot.doty;
	robotState.ddotx = RobotNew.ddotx;
	robotState.ddoty = RobotNew.ddoty;
	char data = '$';
	Uart_write(&EXT1,(char*)&robotState,28);
	Uart_write(&EXT1,(char*)&data,1);
}

void calcPos(void){
	double thetaTemp;

	Robot.vR	 = angVel2LinVel(Robot.wR);
	Robot.vL	 = angVel2LinVel(Robot.wL);

	Robot.xOld    = Robot.x;
	Robot.dotxOld = Robot.dotx;

	Robot.yOld 	  = Robot.y;
	Robot.dotyOld = Robot.doty;

	Robot.thetaOld = Robot.theta;
	if(fabs(Robot.vR - Robot.vL)>1)
		Robot.theta  += ((Robot.vR - Robot.vL)*UPDTIME)/L;

	Robot.w		 = (Robot.vR - Robot.vL)/L;

	Robot.dotx	  =(Robot.x - Robot.xOld)/ UPDTIME;
	Robot.ddotx   = (Robot.dotx - Robot.dotxOld)/UPDTIME;

	if(Robot.dirL == BACKWARD && Robot.dirR == BACKWARD)
		thetaTemp = Robot.theta + M_PI;
	else
		thetaTemp = Robot.theta;

	if(fabs(Robot.vR - Robot.vL)>1){
		Robot.x		 += (L/2)*(Robot.vR + Robot.vL)/(Robot.vR - Robot.vL)*(sin(Robot.theta)-sin(Robot.thetaOld));
		Robot.y		 += (L/2)*(Robot.vR + Robot.vL)/(Robot.vR - Robot.vL)*(cos(Robot.theta)-cos(Robot.thetaOld));
	}
	if(Robot.vR == 0 && Robot.vL == 0){

	}
	else{
		Robot.x 	 += (L/2)*cos(thetaTemp);
		Robot.y 	 += (L/2)*sin(thetaTemp);
	}

	Robot.doty	  = (Robot.y - Robot.yOld)/UPDTIME;
	Robot.ddoty	  = (Robot.doty - Robot.dotyOld)/UPDTIME;

}

void calcPosNew(void){
	RobotNew.theta 	+= (RobotNew.sR - RobotNew.sL)/L;

	RobotNew.xOld    = RobotNew.x;
	RobotNew.dotxOld = RobotNew.dotx;

	RobotNew.yOld 	  = RobotNew.y;
	RobotNew.dotyOld  = RobotNew.doty;

//	Robot.w		 = (Robot.vR - Robot.vL)/L;

	RobotNew.x 	 += RobotNew.s*cos(RobotNew.theta);
	RobotNew.dotx	  =(RobotNew.x - RobotNew.xOld)/ UPDTIME;
	RobotNew.ddotx   = (RobotNew.dotx - RobotNew.dotxOld)/UPDTIME;



	RobotNew.y 	 += RobotNew.s*sin(RobotNew.theta);
	RobotNew.doty	  = (RobotNew.y - RobotNew.yOld)/UPDTIME;
	RobotNew.ddoty	  = (RobotNew.doty - RobotNew.dotyOld)/UPDTIME;

}