#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "uart_intr_c.h"
#include <string.h>

extern Uart_cb 		EXT1;
uint8_t 			returnData[13];
extern bool 		newCom;

volatile struct robot{
	float	sR,sL,s,				//displacement Right, Left, Centerpoint
			wR,wL,
			vR,vL,					//robot's wheels linear velocity
			w,v,					//robot's tangential and angular velocity
			a;						//robot's difference in angular velocities

	int 	tickR,tickL,
			prevTickR,prevTickL;
	float	errSumR,errSumL;
	float   volt;
	float	refvR,refvL;
	float	refPwmR, refPwmL,
			pwmR,pwmL;
	int		dirR,dirL;				//direction of robot's wheel 0 ->forward 1 ->backward
	bool	newCom,					//new command came
			sendOK;
	float	x,y,theta,				//Robot position
			xOld, yOld, thetaOld,
			dotx,doty,
			dotxOld, dotyOld,
			ddotx,ddoty;

}Robot,RobotNew;

struct serial_Data{
	float	x, y, theta,
			dotx,doty,
			ddotx,ddoty;
}robotState;

struct max_vel{
	float tang_max,
		ang_max;
}max_Vel;

struct PID{
	float Kp,Ki;
}PID;

TIM_HandleTypeDef 	motorR,motorL,encoderR,encoderL,motionTimer,serialUpTimer;


#define M_PWM_TickPeriod 	200-1		//period for motor input
#define R				 	85/2		//wheel radius in mm i diastasi kanonika einai 90 meta apo peirama b
#define L				 	144.4		//distance between wheels mm
#define	P				 	1			//prportional term of controller
#define I					0			//inter
#define UPDTIME		 	 	1			//upd every 1ms
#define CPM					1632.67 	// for 1:34.014 gear		464.64//for 1:9.68 gear// gear ratio * encoder resolution
#define PWM_MIN			 	20
#define FORWARD			 	0
#define BACKWARD		 	4
#define STOP			 	1
#define LEFT			 	2
#define RIGHT			 	3
#define MOTION_PERIOD	 	1000 - 1
#define MOTION_PSC		 	SystemCoreClock/2000000 -1
#define SERIALUPLOAD_PERIOD	1000 - 1
#define SERIALUPLOAD_PSC	SystemCoreClock/200000 -1
#define TANG_VEL_MAX		50		//maximum tangential velocity in mm//ergasthrio 8
#define ANG_VEL_MAX			50		//maxinmum angular velocity	  in mm//ergasthrio 18


void PWM_MTy1(TIM_HandleTypeDef *htim, float pwm);
void PWM_MTy2(TIM_HandleTypeDef *htim, float pwm);
void PWM_MTx1(TIM_HandleTypeDef *htim, float pwm);
void PWM_MTx2(TIM_HandleTypeDef *htim, float pwm);
void PWM_MTy1_per(TIM_HandleTypeDef *htim, int PercentWidth);
void PWM_MTy2_per(TIM_HandleTypeDef *htim, int PercentWidth);
void PWM_MTx1_per(TIM_HandleTypeDef *htim, int PercentWidth);
void PWM_MTx2_per(TIM_HandleTypeDef *htim, int PercentWidth);
void MT_Input_Init(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void EncoderInit(TIM_HandleTypeDef *encoderL,TIM_HandleTypeDef *encoderR);
void Stop(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR);
void MoveF(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR, int PercentWidth);
void MoveB(TIM_HandleTypeDef *motorL,TIM_HandleTypeDef *motorR, int PercentWidth);
void StopR(TIM_HandleTypeDef *motorR);
void StopL(TIM_HandleTypeDef *motorL);
float tick2wheelRev(int tick);
float wheelRev2tick(float cpr);
float wheelDist(float cpr);
float dist2wheelRev(float dist);
void motion(uint16_t x,uint16_t r, int dir);
float vel2pwm(float w);
float pwm2vel(float pwm);
void updateRobot(void);
void updateRobotNew(void);
void Pcontrol(void);
void PIDcontrolNew(void);
void MoveRF(TIM_HandleTypeDef *motor);
void MoveLF(TIM_HandleTypeDef *motor);
void moveRobot(void);
void MOTION_TIM(TIM_HandleTypeDef *htim);
void MotorInit(void);
void moveRobotHandler(int x, int r, int dir);
void robotSetPos(float x, float y, float theta);
void SendData(void);
float angVel2LinVel(float w);
void calcPos(void);
void calcPosNew(void);
void robotSetPos(float x, float y, float theta);
void movePanoramixHandle(double tangVel, double angVel);
void movePanoramixTeleopKeyHandle(double tangVel, double angVel);
