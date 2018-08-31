#include "main.h"

Uart_cb 		EXT1;
uint8_t			a[64];
char 			packet[20]={0};
char 			data[300];
bool			newCom = true;

void main()
{
	HAL_Init();
	uartInit(&EXT1);
	MotorInit();
	EXT1.special_end_char='\n';
	updateRobot();
	calcPos();
//	SendData();
	while(1);
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

//* UART handle msgs from turtlesim teleop
void UART_RxCpltCallback(Uart_cb *uart){
	char* arrayIn;
	float* temp;
	float tangVel, 			//tangential velocity
			angVel;				//angular velocity
	while(EXT1.Rxbuffer.numOfLines){
		Uart_read_line(&EXT1,packet,sizeof(packet));
		arrayIn = packet;
		if(*arrayIn== '!'){
			newCom = false;
			temp = arrayIn+1;
			tangVel =*temp;
			angVel = *(temp+1);
			Robot.newCom = true;	//new move arrayIn
			if((int)angVel==0){
				if((int)tangVel>0) movePanoramixTeleopKeyHandle(max_Vel.tang_max,0);
				else if((int)tangVel<0) movePanoramixTeleopKeyHandle(-max_Vel.tang_max,0);
				else movePanoramixTeleopKeyHandle(0,0);
			}
			else if((int)angVel>0) movePanoramixTeleopKeyHandle(0,max_Vel.ang_max);
			else movePanoramixTeleopKeyHandle(0,-max_Vel.ang_max);
		}
		else if(*arrayIn == '&'){
			newCom = false;
			temp = arrayIn+1;
			tangVel = *temp;
			angVel 	= *(temp+1);
			Robot.newCom = true;
			movePanoramixHandle(tangVel,angVel);
		}
		else if(*arrayIn == '@'){
			temp = arrayIn+1;
			PID.Kp = *(temp);
			PID.Ki = *(temp+1);
		}
		else if(*arrayIn == '#'){
			temp = arrayIn+1;
			max_Vel.tang_max = *(temp);
			max_Vel.ang_max = *(temp+1);
			if(Robot.refPwmL == 0 && Robot.refPwmR == 0){}
			else if(Robot.refPwmL == 0){
				Robot.refPwmR = vel2pwm(LinVel2angVel(max_Vel.ang_max));
			}
			else if(Robot.refPwmR == 0){
				Robot.refPwmL = vel2pwm(LinVel2angVel(max_Vel.ang_max));
			}
			else{
				Robot.refPwmR = vel2pwm(LinVel2angVel(max_Vel.tang_max));
				Robot.refPwmL = Robot.refPwmR;
			}
		}
		else if(*arrayIn == '$'){
//			SendData();
			Robot.sendOK = true;
		}
		else if(*arrayIn == '%'){
			temp = arrayIn+1;
			robotSetPos(*temp,*(temp+1),*(temp+2));
		}
		else if(*arrayIn == '^'){
			temp = arrayIn+1;
			RobotNew.a = *temp;
		}
	}
}
//*/

/*		First uart handle
void UART_RxCpltCallback(Uart_cb *uart)
{
	uint16_t *mat16;
	uint32_t *mat32;
	int dataSize=6;
	//kaleite kathe fora poy yarxei special charcacter stin periptosi mou to /n
	while(EXT1.Rxbuffer.numOfLines)//an den exw prolavei na kanw parce mia entoli tin trexei mazi me tin epomeni
	{
		Uart_read_line(&EXT1,packet,dataSize);

		if (packet[0]=='!')
		{
			//it means that the packet is valid. All valid packets
			if (packet[1]==0x01)									//packet[1] == 1 move robot
			{
				//mat16 = &packet[2];
				//moveRobotHandler(mat16[0],mat16[1],mat16[2]);
				moveRobotHandler(packet[2],packet[3],packet[4]);
			}
			else if(packet[1]==0x02)
			{
				SendData();
			}
			else if(packet[1]==0x03)
			{
				//mat32 = &packet[2];
				//robotSetPos(mat32[0],mat32[1],mat32[2]);
				robotSetPos(packet[2],packet[3],packet[4]);
			}
		}
	}
}
//*/
