/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/



//for get the relectance value 
struct sensors_ checkSensors(void) {
	struct sensors_ ref;
	struct sensors_ dig;
	// read raw sensor values
    	reflectance_read(&ref);
    	// print out each period of reflectance sensors
    	printf("%5d %5d %5d %5d %5d %5d\r\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);  	 
   	 
    	// read digital values that are based on threshold. 0 = white, 1 = black
    	// when blackness value is over threshold the sensors reads 1, otherwise 0
    	reflectance_digital(&dig);
    	//print out 0 or 1 according to results of reflectance period
    	printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
    	return dig;
}
 

// set the case for the turning system
void left(int level, int speedL, int speedR) {
 switch(level) {
	case 1:
	speedR = speedR + speedR/2;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
    
	break;
	case 2:
	speedR = speedR + speedR/2;
	speedL = speedL - speedL/3;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
	case 4:
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(0);
	PWM_WriteCompare2(speedR);
	break;
	case 3:
	speedR = speedR + speedR/2;
	speedL = speedL - speedL/2;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
	case 5:
	speedR = 2*speedR;
	MotorDirLeft_Write(1);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
}
}

void right(int level, int speedL, int speedR) {
 switch(level) {
	case 1:
	speedL = speedL + speedL/2;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
    
	break;
	case 2:
	speedL = speedL + speedL/2;
	speedR = speedR - speedR/3;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
	case 4:
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(0);
	break;
	case 3:
	speedL = speedL + speedL/2;
	speedR = speedR - speedR/2;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(0); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
	case 5:
	speedL = 2*speedL;
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
	MotorDirRight_Write(1); 	// set RightMotor forward mode
	PWM_WriteCompare1(speedL);
	PWM_WriteCompare2(speedR);
	break;
}
}

void sendPos(int posX, int posY) {
	print_mqtt("Zumo016/debug", "X Position: %d", posX);
	print_mqtt("Zumo016/debug", "Y Position: %d", posY);
}

void lineFollower(struct sensors_ dig, int speedL, int speedR) {
	if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 1 && dig.r3 == 1) {
        	right(3, speedL, speedR);
    	}
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 1 && dig.r3 == 0) {
        	right(3, speedL, speedR);
    	}
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 1) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	left(3, speedL, speedR);  
    	}
    	else if(dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	left(3, speedL, speedR);  
    	}
    	else if(dig.l3 == 1 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	left(4, speedL, speedR);  
    	}
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0) {
   		 MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR);  
    	}
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	left(1, speedL, speedR);  
    	}
    	else if(dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	left(3, speedL, speedR);
    	}
    	/*else if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0) {
        	//left(5, speedL, speedR);
    	}*/
    	/*else if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0) {
        	//left(5, speedL, speedR);
    	}*/
    	else if(dig.l3 == 0 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0) {
        	left(4, speedL, speedR);
    	}
   	 else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 0 && dig.r3 == 0) {
        	right(1, speedL, speedR);
    	}
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 0) {
        	right(3, speedL, speedR);
    	}
    	/*else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 0 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1) {
        	//right(5, speedL, speedR);
    	}*/
    	/*else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1) {
        	//right(5, speedL, speedR);
    	}*/
    	else if(dig.l3 == 0 && dig.l2 == 0 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 0) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l3==1 && dig.r3==0) {
        	left(4, speedL, speedR);
    	}
    	else if(dig.l1==1 && dig.r1==0) {
        	left(4, speedL, speedR);
    	}
    	else if(dig.l3==0 && dig.r3==1) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l1==0 && dig.r1==1) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l3==1 && dig.r3==1) {
       	 
    	}
    	else if(dig.l1==1 && dig.r1==1) {
       	 
    	}
    	else {
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(0);
        	PWM_WriteCompare2(0);  
    	}
}

#if 0
// Alex Maze
    
void zmain(void)
{
	int lines = 0;
	struct sensors_ dig;
    
	int x[7] = {0,0,0,1,0,0,0};
	int posX;
	int y[12] = {1,0,0,0,0,0,0,0,0,0,0,0};
	int posY;
	int direction = 0;
	int last = 6;
	int buttom = 0;
	int speedL = 70;
	int speedR = 70;
	int distance;
    
	Ultra_Start();
	motor_start();
   motor_forward(0,0);
   IR_Start();
   IR_flush();
   reflectance_start();
   reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000

	//x[4]=1;
	//y[0]=1;
   
    
    
	while(buttom==0) {
    	lineFollower(dig, speedL, speedR);
    	if(SW1_Read() == 0) {
        	send_mqtt("Zumo016/debug", "Boot");
        	buttom++;
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR);
       	 
        	for(;;) {
        	dig = checkSensors();
     	if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1) {
            	motor_forward(0,0);
            	break;
        	}   
    	}   
    	}
	}
    
	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR);

	for(;;) {
	print_mqtt("Zumo016/debug", "Direction: %d", direction);
	distance = Ultra_GetDistance();
	printf("distance = %d\r\n", distance);
	dig = checkSensors();
	if(distance<12 && distance!=0) {
    	print_mqtt("Zumo016/debug", "Block Detected: %d", distance);
    	if(direction==0){
    	if(rand()%2==0) {
       	right(6, speedL, speedR);
    	vTaskDelay(500);
        	direction++;
    	} else {
        	left(6, speedL, speedR);
        	vTaskDelay(500);
        	direction--;
    	}
    	}
    	else if(direction<0) {
     	right(6, speedL, speedR);
    	vTaskDelay(500);
    	direction++;
    	}
    	else if(direction>0) {
     	left(6, speedL, speedR);
    	vTaskDelay(500);
    	direction--;
    	}
    	lines = 0;
    	for(int z=0; z<12; z++) {
        	if(y[z]==1) {
            	y[z]=0;
            	y[z+1]=1;
            	posY=z;
            	break;
        	}
    	}
	}
 
	else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && direction==0 && last<6) {
    	print_mqtt("Zumo016/debug", "Line detected and good direction");
    	for(int z=0; z<12; z++) {
        	if(y[z]==1) {
            	y[z]=0;
            	y[z+1]=1;
            	posY=z;
            	break;
        	}
    	}
	}
    
	else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && direction<0 && last<6) {
    	if(lines==1){
    	print_mqtt("Zumo016/debug", "Block Skiped, turning to the right");
    	for(int z=0; z<7; z++) {
        	if(x[z]==1) {
            	x[z]=0;
            	x[z-1]=1;
            	direction++;
            	posX=z;
            	break;
        	}
    	}
    	right(5, speedL, speedR);
    	vTaskDelay(600);
    	}
    	lines++;
	}
    
	else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && direction>0 && last<6) {
    	if(lines==1) {
    	print_mqtt("Zumo016/debug", "Block Skiped, turning to the right");
    	for(int z=0; z<7; z++) {
        	if(x[z]==1) {
            	x[z]=0;
            	x[z+1]=1;
            	direction--;
            	posX=z;
            	break;
        	}
    	}
    	left(5, speedL, speedR);
    	vTaskDelay(600);
    	}
    	lines++;
	}
    
	if(y[11]==1 && direction==0 && posX<3) {
    	print_mqtt("Zumo016/debug", "I am in y=11 and posX<3");
    	right(5, speedL, speedR);
    	direction++;
	}
	if(y[11]==1 && direction==0 && posX>3) {
    	print_mqtt("Zumo016/debug", "I am in y=11 and posX>3");
    	left(5, speedL, speedR);
    	direction--;
	}
	if(y[11]==1 && direction<0 && posX==3) {
    	print_mqtt("Zumo016/debug", "I am in y=11 and posX==3 and go to the right");
    	right(5, speedL, speedR);
    	direction++;
	}
	if(y[11]==1 && direction>0 && posX==3) {
    	print_mqtt("Zumo016/debug", "I am in y=11 and posX==3 and go to the left");
    	left(5, speedL, speedR);
    	direction--;
	}
	lineFollower(dig, speedL, speedR);
	print_mqtt("Zumo016/debug", "Line Follower");
	last = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;
	sendPos(posX, posY);
	print_mqtt("Zumo016/debug", "End of loop");
	}
 }   
#endif



#if 0
    
//MAZE
void zmain(void)
{
	struct sensors_ dig;
    
	//uint32_t IR_val;
    
    bool start = false; //if start false print 1 time and change to true
    bool end = false;  //if end false print 1 time and change to true
    bool online = false;  //if online false print 1 time and change to true
	
    int lines = 0;  //read the line
	int last = 6;  // read the sensors
	int button = 0; // for the button
	int speedL = 120; //left motor speed
	int speedR = 112; //right motorspeed
    uint32_t IR_val; //for the IR value
    int x[12]={1,2,3,4,5,6,7,8,9,10,11,12};// array for possion x
    int a =1;
    int y[7] ={1,2,3,4,5,6,7};// array for possion y
    int b=4;
	motor_start(); //motor start
	motor_forward(0,0);
	IR_Start(); 
	IR_flush();
    Ultra_Start();
	reflectance_start();
	reflectance_set_threshold(7000, 7000, 18000, 18000, 7000, 7000); // set center sensor threshold to 11000 and others to 9000
   
    // for the button if the button go forward. 
	while(button==0) {
    	if(SW1_Read() == 0) {
        	button++;
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(150);
        	PWM_WriteCompare2(142);
       	
            //when get on line motor stop.
        	for(;;) {
                //read from sensors.
        	dig = checkSensors();
     	if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines==0) {
            	 motor_stop();
    
                Beep(200,100);
            
            	break;
        	}   
    	}   
    	}
       
	}
    print_mqtt("Zumo016/ready", "line");
    IR_wait();
    motor_start();
    
    //int a value for read the distance
    int d;
    //int ticktype to recall time.
    TickType_t Start,End,Time;
 
    //when get IR singal go forward.
    while(IR_get(&IR_val, portMAX_DELAY))
    if(start == false)
	{
        start = true;
    print_mqtt("Zumo016/start", "%d", xTaskGetTickCount());
    break;
    }
           
    for(;;) 
    {
        //read from sensors and ultra value.
            dig = checkSensors();
            d = Ultra_GetDistance();
        //when the posion is on the middle of the line go forward.
            if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1&&d>20&&b==4)
        {
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
            a++;
        }
        
        //when detected blocker turn left
        else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1&&d<20)
        {
            MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);
            b--;
            
        
        for (b=4;b<4;b--)
        {
            //after turn go forward
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
            b--;
        //when reach the next line try to turn right 
            if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1)
        {
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(1); 	// set RightMotor forward mode
        	PWM_WriteCompare1(160);
        	PWM_WriteCompare2(70);
            a++;
            
            //if there have block turn laft
            if(d<20)
            {
            MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);
            b--;
            ]
            
            //if nothing go forward
            else
            {   
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
            a++
            }
        }
        
        //when get posion a = 9 turn right
        else if (a==9)
        {   MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(1); 	// set RightMotor forward mode
        	PWM_WriteCompare1(160);
        	PWM_WriteCompare2(70);
            b++;
        }
        
        //when get posion b = 4 turn left
        else if (b==4)
        {
            MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);
            a++;
        }
        
        //when at b = 4 detected block turn right
        else if (b==4&&d<20)
        {   MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(1); 	// set RightMotor forward mode
        	PWM_WriteCompare1(160);
        	PWM_WriteCompare2(70);
            b++;
        for (b=4;b>4;b++)
        {
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
            b++;
            
            if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1&&b==6)
        {
             MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);
            a++;
            }
        else if (a==11)
        {    MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);

            b--;
        }
        else if (b==4)
        {
            MotorDirLeft_Write(1);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(60);
        	PWM_WriteCompare2(160);
            a++;
        }
        
        
        }
        }
        }
        // if online go straight
        else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
   		    MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
           
    	}
        //if line on right turn right with highest left speed
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==1 && dig.r3==1) {
        	right(5, speedL, speedR);
    	}
        else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==1) {
        	right(5, speedL, speedR);
    	}
        //if line on the right-mid turn right whit higher left speed
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==1 && dig.r3==0) {
        	right(3, speedL, speedR);
    	}
        //if line on letf turn left with highest right speed
    	else if(dig.l3==1 && dig.l2==1 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(5, speedL, speedR);  
    	}
        else if(dig.l3==1 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(5, speedL, speedR);  
    	}
        else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(5, speedL, speedR);
    	}
        else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	left(5, speedL, speedR);
    	}
        //if line on left-mid turn left with higher right speed
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(3, speedL, speedR);  
    	}
        //if line on mid-letf turn left with high right speed
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(1, speedL, speedR);  
    	}
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(2, speedL, speedR);
    	}
        //when get a turning line read left-mid and mid sensor turn left
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	left(4, speedL, speedR);
    	}
        else if(dig.l3==1 && dig.r3==0) {
        	left(4, speedL, speedR);
    	}
    	else if(dig.l1==1 && dig.r1==0) {
        	left(4, speedL, speedR);
    	}
        //turn right when mid-right on line with high speed
   	    else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	right(1, speedL, speedR);
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==1 && dig.r3==0) {
        	right(2, speedL, speedR);
    	}
        //if turning line on right turn right with highest left speed
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==1 && dig.r3==1) {
        	right(5, speedL, speedR);
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1) {
        	right(5, speedL, speedR);
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==0) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l3==0 && dig.r3==1) {
        	right(4, speedL, speedR);
    	}
    	else if(dig.l1==0 && dig.r1==1) {
        	right(4, speedL, speedR);
    	}
        //when line off motor stop
        else {
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(0);
        	PWM_WriteCompare2(0);  
    	}
    }




}

#endif
#if 0
//Follow the line
void zmain(void)
{
	struct sensors_ dig;
    
	//uint32_t IR_val;
    
    bool start = false; //if start false print 1 time and change to true
    bool end = false;  //if end false print 1 time and change to true
    bool online = false;  //if online false print 1 time and change to true
	int lines = 0;  //read the line
	int last = 6;  // read the sensors
	int button = 0; // for the button
	int speedL = 120; //left motor speed
	int speedR = 112; //right motorspeed
    uint32_t IR_val; //for the IR value
    
	motor_start(); //motor start
	motor_forward(0,0);
	IR_Start(); 
	IR_flush();
	reflectance_start();
	reflectance_set_threshold(7000, 7000, 18000, 18000, 7000, 7000); // set center sensor threshold to 11000 and others to 9000
   
    
	while(button==0) {
    	if(SW1_Read() == 0) {
        	button++;
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(150);
        	PWM_WriteCompare2(142);
       	 
        	for(;;) {
        	dig = checkSensors();
     	if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines==0) {
            	 motor_stop();
    
                Beep(200,100);
            
            	break;
        	}   
    	}   
    	}
       
	}
    print_mqtt("Zumo016/ready", "line");
    IR_wait();
    motor_start();
    TickType_t Start,End,Time;
 
    
    while(IR_get(&IR_val, portMAX_DELAY))
    if(start == false)
	{
        start = true;
    print_mqtt("Zumo016/start", "%d", xTaskGetTickCount());
    break;
    }
           
    for(;;) 
    {

            dig = checkSensors();
            // for read and count the line
            if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines<3 && last<6)
            {
                lines++;
          	 
        	}
            //stop at the second line
            else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines==2&& last==6)
        {
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(0);
        	PWM_WriteCompare2(0);
            if(end == false)
            {
            end = true;
            End = xTaskGetTickCount();
            Time = End - Start;
            
            
   	        print_mqtt("Zumo016/stop", "%d",End);
            print_mqtt("Zumo016/time", "%d",Time);
            }

            break;
  
	}
            //keep going forward
    	else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines<4 && last==6) {
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(150);
        	PWM_WriteCompare2(142);  
    	}

        //turn right with highest left speed.
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==1 && dig.r3==1) {
        	right(5, speedL, speedR);
            
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
        else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==1) {
        	right(5, speedL, speedR);
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
            
    	}
        else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==1 && dig.r3==1) {
        	right(5, 1.2*speedL, speedR);
           
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1) {
        	right(5, speedL, speedR);
            if(online == true)
            {
                online = false;
            print_mqtt("Zumo016/online", "%d",xTaskGetTickCount());
            }
            
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==0) {
        	right(4, speedL, 0.5*speedR);
            if(online == true)
            {
                online = false;
            print_mqtt("Zumo016/online", "%d",xTaskGetTickCount());
            }
    	}
        
        // turn right with higher left speed.
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==1 && dig.r3==0) {
        	right(3, speedL, speedR);
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
        
    	//turn left with higher right speed.
    	else if(dig.l3==1 && dig.l2==1 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(3, speedL, speedR);  
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(3, speedL, speedR); 
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
        
        
        //turn left with highest right speed.
    	else if(dig.l3==1 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(5, speedL, speedR);  
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
        else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(5, 0,speedR);
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
    	else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	left(5, 0, speedR);
         if(online == true)
            {
                online = false;
            print_mqtt("Zumo016/online", "%d",xTaskGetTickCount());
            }
    	}
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	left(4, 0.5*speedL, speedR);
            if(online == true)
            {
                online = false;
            print_mqtt("Zumo016/online", "%d",xTaskGetTickCount());
            }
    	}
        else if(dig.l3==1 && dig.r3==0) {
        	left(4, speedL, speedR);
    	}
    	else if(dig.l1==1 && dig.r1==0) {
        	left(4, speedL, speedR);
    	}

        
        //on straight line go straight
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
   		 MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(speedL);
        	PWM_WriteCompare2(speedR); 
            if(online == true)
            {
                online = false;
            print_mqtt("Zumo016/online", "%d",xTaskGetTickCount());
            }
    	}
        
        
        //turn left with little high right speed.
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(1, speedL, speedR);  
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
    	else if(dig.l3==0 && dig.l2==1 && dig.l1==1 && dig.r1==0 && dig.r2==0 && dig.r3==0) {
        	left(2, 0.5*speedL, speedR);
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
    	
        //turn right with little high left speed.
   	    else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==0 && dig.r3==0) {
        	right(1, speedL, speedR);
            if(online == false)
            {
                online = true;
            print_mqtt("Zumo016/miss", "%d",xTaskGetTickCount());
            }
    	}
    	else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==1 && dig.r3==0) {
        	right(2, speedL, speedR/4);
           
    	}
    	
        //when out of line motor stop.
    	else {
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(0);
        	PWM_WriteCompare2(0);  
    	}
    	last = dig.l3+dig.l2+dig.l1+dig.r1+dig.r2+dig.r3;
    	printf("%d %d \n", lines, last);
        
        
	}    
 }

#endif



#if 1

    
    //SUMO
void zmain(void)
{   
    
    bool start = false; //if start false print 1 time and change to true
    bool end = false;  //if end false print 1 time and change to true
    bool online = false;  //if online false print 1 time and change to true
	bool line= false;
    bool hit = true;
    bool send =true;
    int lines = 0;  //read the line
	int last = 6;  // read the sensors
	int button = 0; // for the button
	int speedL = 120; //left motor speed
	int speedR = 112; //right motorspeed
    uint32_t IR_val; //for the IR value
    struct sensors_ dig;
    struct accData_ data;

	motor_start();
	motor_forward(0,0);
	IR_Start();
	IR_flush();
	reflectance_start();
	reflectance_set_threshold(9000, 9000, 20000, 20000, 9000, 9000); // set center sensor threshold to 20000 and others to 9000
    
    //when press the button go forward reach the line motor stop.
	while(button==0) 
    {
    
    	if(SW1_Read() == 0) 
        {
        	button++;
        	MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(0); 	// set RightMotor forward mode
        	PWM_WriteCompare1(180);
        	PWM_WriteCompare2(172);
       	 
        	for(;;) {
        	            dig = checkSensors();
            //when reach the line motor stop
     	                if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 && dig.r2==1 && dig.r3==1 && lines==0) 
                        {
            	            motor_stop();
    
                            Beep(200,100);
            	            lines++;
            	            break;
        	            }   
    	            }   
        }
	}
    //wait IR
        IR_wait();

        motor_start();
        
   
        if(line==false)
        {line=true;
         print_mqtt("Zumo016/ready", "line");
       	}
        TickType_t Start,End,Time;
       
       	while(IR_get(&IR_val, portMAX_DELAY))
        {
                
            if(start == false)
	            {
                      start = true;
                      Start = xTaskGetTickCount();
                      print_mqtt("Zumo016/start", "%d", xTaskGetTickCount());
                      
                }
  
        	             MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	             MotorDirRight_Write(0); 	// set RightMotor forward mode
        	             PWM_WriteCompare1(180);
        	             PWM_WriteCompare2(172);
                         vTaskDelay(200);
        
                        
        //for Sumo when get IR siganl go forward when reach the line revers and then turn back then go forward.
        for(;;)
            {
                    
                	dig = checkSensors();

                if(dig.l3==1 || dig.l2==1 || dig.l1==1 || dig.r1==1 || dig.r2==1 || dig.r3==1 ) //when check the line backward and turn
                    {
                        MotorDirLeft_Write(1);  	
                    	MotorDirRight_Write(1); 	
                    	PWM_WriteCompare1(190);
                    	PWM_WriteCompare2(182);
                        vTaskDelay(200);
                        MotorDirLeft_Write(1);  	
                    	MotorDirRight_Write(0); 	
                    	PWM_WriteCompare1(190);
                    	PWM_WriteCompare2(182);
                        vTaskDelay(500);

                    }
                    
                    // when on the white paper go forward
                else if (dig.l1==0||dig.r1==0)// go forward
                    {
                        MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	             MotorDirRight_Write(0); 	// set RightMotor forward mode
        	             PWM_WriteCompare1(180);
        	             PWM_WriteCompare2(172);
  
                    }
                //when dective hit print mqtt for hit and time and set a switch to measure the sending MQTT message
                    else if(data.accX<-500&&send==true)
                    {   send = false;
                       //send the message onle 1 time by a hit infor switch
                        if(hit == true)
                    { hit = false;
                        print_mqtt("Zumo016/hit", "%d", xTaskGetTickCount());
                    }
                    else if(hit ==false)
                    {
                        hit = true;  
                    }
                    }
                    else if (send == false)
                    {
                        send = true;
                    }
 
                    //when press the button again motor stop and print the time
                if( SW1_Read() == 0 )//push button to stop the motor.
                    {
                       
                        motor_stop();
                        End = xTaskGetTickCount();
                        Time = End - Start;
                        print_mqtt("Zumo016/stop", "%d",End);
                        print_mqtt("Zumo016/time", "%d",Time);
                        break;
                    } 
                 
            }
          
        
        }

}
 
#endif
    


#if 0
// Name and age
void zmain(void)
{
    char name[32];
    int age;
    
    
    printf("\n\n");
    
    printf("Enter your name: ");
    //fflush(stdout);
    scanf("%s", name);
    printf("Enter your age: ");
    //fflush(stdout);
    scanf("%d", &age);
    
    printf("You are [%s], age = %d\n", name, age);

    while(true)
    {
        BatteryLed_Write(!SW1_Read());
        vTaskDelay(100);
    }
 }   
#endif


#if 0
//battery level//
void zmain(void)
{
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    for(;;)
    {
        char msg[80];
        ADC_Battery_StartConvert(); // start sampling
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        vTaskDelay(500);
       
    #define coeff 25/4095/3;/* code number from 0 to max read 4095,ADC read 3 of 5 of the total resistance */
    UART_1_Start();
    ADC_Battery_Start();
    ADC_Battery_StartConvert();
    int16_t value=0, volatge;
    printf("BETTRY.\n");
    for(;;)
    { 
       ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT);/* For the bettry */
       value =ADC_Battery_GetResult16();
       volatge = value*coeff;
       printf("%d\n",volatge);
       if (volatge<=4)
        
        { 
     BatteryLed_Write(1);
    vTaskDelay(500);
   
    Beep(500,100);
    vTaskDelay(500);
    
        }
    }
 }   
#endif

#if 0
// button
void zmain(void)
{
    while(1) {
        printf("Press button within 5 seconds!\n");
        int i = 50;
        while(i > 0) {
            if(SW1_Read() == 0) {
                break;
            }
            vTaskDelay(100);
            --i;
        }
        if(i > 0) {
            printf("Good work\n");
            while(SW1_Read() == 0) vTaskDelay(10); // wait until button is released
        }
        else {
            printf("You didn't press the button\n");
        }
    }
}
#endif

#if 0
// button
void zmain(void)
{
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    for(;;)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
               int i;
        for(i=0;i<3;i++)
        {
     BatteryLed_Write(1);
    vTaskDelay(500);
    BatteryLed_Write(0);
    vTaskDelay(500);
        }
        for (i=0;i<3;i++)
        {
    BatteryLed_Write(1);
    vTaskDelay(1500);
    BatteryLed_Write(0);
    vTaskDelay(500);
    BatteryLed_Write(1);
    }
    for(i=0;i<3;i++)
        {
     BatteryLed_Write(1);
    vTaskDelay(500);
    BatteryLed_Write(0);
    vTaskDelay(500);
        }
     for (i=0;i<3;i++)
    {
    BatteryLed_Write(1);
    vTaskDelay(1500);
    BatteryLed_Write(0);
    vTaskDelay(500);
    BatteryLed_Write(1);
    Beep(1000, 150);
    while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
            }
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
void zmain(void)
{
    Ultra_Start();                          // Ultra Sonic Start function
    
    while(1) {
        int d = Ultra_GetDistance();
        // Print the detected distance (centimeters)
        printf("distance = %d\r\n", d);
        vTaskDelay(200);
        if  (d<10)
    {
        Beep(1000, 150);
        motor_backward(150,1000);// moving backward
        Beep(1000, 150);
        motor_turn(200,50,600);
    }
    
    }
    
}   
#endif

#if 0
//IR receiverm - how to wait for IR remote commands
void zmain(void)
{
    IR_Start();
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    bool led = false;
    // Toggle led when IR signal is received
    for(;;)
    {
        IR_wait();  // wait for IR command
        led = !led;
        BatteryLed_Write(led);
        if(led) printf("Led is ON\n");
        else printf("Led is OFF\n");
    }    
 }   
#endif



#if 0
//IR receiver - read raw data
void zmain(void)
{
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    // print received IR pulses and their lengths
    for(;;)
    {
        if(IR_get(&IR_val, portMAX_DELAY)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
        }
    }    
 }   
#endif


#if 0
//reflectance
  

  void zmain(void)

{
    struct sensors_ ref;
    struct sensors_ dig;
 
    
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    for(;;)
    {
        // read raw sensor values
        reflectance_read(&ref);
        // print out each period of reflectance sensors
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig); 
        //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);        
        
        vTaskDelay(200);
    }
    
    
}   
#endif


#if 0
//motor
void zmain(void)
{
    motor_start();              // enable motor controller
    motor_forward(0,0);         // set speed to zero to stop motors

    vTaskDelay(1000);
    
  
            MotorDirLeft_Write(0);  	// set LeftMotor forward mode
        	MotorDirRight_Write(1); 	// set RightMotor forward mode
        	PWM_WriteCompare1(160);
        	PWM_WriteCompare2(70);
            vTaskDelay(560);
    
   motor_stop(); 
    
    
 
    
    motor_turn(00,000,0000);     // turn
    motor_backward(000,0000);    // moving backward
     
    motor_forward(0,0);         // stop motors

    motor_stop();               // disable motor controller
    
    for(;;)
    {

    }
}
#endif

#if 0
/* Example of how to use te Accelerometer!!!*/
    
   int random_number();
   int number;
   void zmain(void)
    
{
    struct accData_ data;
    Ultra_Start();
   motor_start();
   motor_forward(0,0);
   vTaskDelay(500);
     //forward
        MotorDirLeft_Write(0);      
        MotorDirRight_Write(0);
        PWM_WriteCompare1(200); 
        PWM_WriteCompare2(191);
    
      

   
   
    
    printf("Accelerometer test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Device Ok...\n");
    }
    
    for(;;)
    {
        LSM303D_Read_Acc(&data);
        printf("%8d %8d %8d\n",data.accX, data.accY, data.accZ);
        vTaskDelay(50);
    
    
  
    if (data.accX<-2000)
    {
 //0=left, 1 =right
        number=random_number();
if (number==1)
    {
        motor_forward(0,0);
    	Beep(200,50);
    	MotorDirLeft_Write(1); 	 
    	MotorDirRight_Write(1);	 
    	PWM_WriteCompare1(150);
    	PWM_WriteCompare2(50);
    	vTaskDelay(700);
	}
	else if (number==0) 
     {
        motor_forward(0,0);
    	Beep(200,50);
    	MotorDirLeft_Write(1); 	 
    	MotorDirRight_Write(1);	 
    	PWM_WriteCompare1(50);
    	PWM_WriteCompare2(150);
    	vTaskDelay(700);
     }   
    }
        MotorDirLeft_Write(0);      
        MotorDirRight_Write(0);
        PWM_WriteCompare1(200); 
        PWM_WriteCompare2(191);
       
  
        }
}
 int random_number()
{
    uint8_t count = 0;
	int random_number=rand() %2;
	if(random_number)
	{
 	    printf("Right\n");
	}
	else
	{
        count++;
    	printf("Left %d \n", count);
	}
	return random_number;
}
   
#endif    

#if 0
// MQTT test
void zmain(void)
{
    int ctr = 0;

    printf("\nBoot\n");
    send_mqtt("Zumo016/debug", "Boot");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 

    for(;;)
    {
        printf("Ctr: %d, Button: %d\n", ctr, SW1_Read());
        print_mqtt("Zumo016/debug", "Ctr: %d, Button: %d", ctr, SW1_Read());

        vTaskDelay(1000);
        ctr++;
    }
 }   
#endif


#if 0
void zmain(void)
{    
    struct accData_ data;
    struct sensors_ ref;
    struct sensors_ dig;
    
    printf("MQTT and sensor test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Accelerometer Ok...\n");
    }
    
    int ctr = 0;
    reflectance_start();
    for(;;)
    {
        LSM303D_Read_Acc(&data);
        // send data when we detect a hit and at 10 second intervals
        if(data.accX > 1500 || ++ctr > 1000) {
            printf("Acc: %8d %8d %8d\n",data.accX, data.accY, data.accZ);
            print_mqtt("Zumo01/acc", "%d,%d,%d", data.accX, data.accY, data.accZ);
            reflectance_read(&ref);
            printf("Ref: %8d %8d %8d %8d %8d %8d\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
            print_mqtt("Zumo01/ref", "%d,%d,%d,%d,%d,%d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
            reflectance_digital(&dig);
            printf("Dig: %8d %8d %8d %8d %8d %8d\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            print_mqtt("Zumo01/dig", "%d,%d,%d,%d,%d,%d", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            ctr = 0;
        }
        vTaskDelay(10);
    }
 }   

#endif

#if 0
void zmain(void)
{    
    RTC_Start(); // start real time clock
    
    RTC_TIME_DATE now;

    // set current time
    now.Hour = 10;
    now.Min = 10;
    now.Sec = 56;
    now.DayOfMonth = 29;
    now.Month = 11;
    now.Year = 2018;
    RTC_WriteTime(&now); // write the time to real time clock

    for(;;)
    {
        if(SW1_Read() == 0) {
            // read the current time
            RTC_DisableInt(); /* Disable Interrupt of RTC Component */
            now = *RTC_ReadTime(); /* copy the current time to a local variable */
            RTC_EnableInt(); /* Enable Interrupt of RTC Component */
`
            printf("%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
            print_mqtt("Zumo016", "%2d:%02d.%02d",now.Hour, now.Min, now.Sec);
            vTaskDelay(50);
            while(SW1_Read() == 0) 
            {}
        }
        vTaskDelay(50);
    }
    //Timer for exercise 1.2


}
  

    
#endif


/* [] END OF FILE */
