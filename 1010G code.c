#pragma config(Sensor, dgtl1,  encoder_left,   sensorQuadEncoder) // [drive_right] encoder
#pragma config(Sensor, dgtl3,  encoder_right,  sensorQuadEncoder) // [drive_left] encoder
#pragma config(Motor,  port1,           claw,          tmotorVex393_HBridge, openLoop) // claw
#pragma config(Motor,  port2,           drive_left,    tmotorVex393HighSpeed_MC29, openLoop) // drive left
#pragma config(Motor,  port3,           drive_right,   tmotorVex393HighSpeed_MC29, openLoop, reversed) // drive right
#pragma config(Motor,  port4,           mobile_goal_intake, tmotorVex393_MC29, openLoop) // mobile goal intake
#pragma config(Motor,  port5,           lift_top_left, tmotorVex393_MC29, openLoop) // lift top left
#pragma config(Motor,  port6,           lift_bottom_left, tmotorVex393_MC29, openLoop) // lift bottom left
#pragma config(Motor,  port7,           lift_top_right, tmotorVex393_MC29, openLoop, reversed) // lift top right
#pragma config(Motor,  port8,           lift_bottom_right, tmotorVex393_MC29, openLoop, reversed) // lift bottom right
#pragma config(Motor,  port9,           left_chainbar, tmotorVex393_MC29, openLoop) // left chainbar
#pragma config(Motor,  port10,          right_chainbar, tmotorVex393_HBridge, openLoop, reversed) // right chainbar


#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"


void pre_auton()
{
	bStopTasksBetweenModes = true;
}



task autonomous()	 //autonomous

{ // begin autonomous
	motor(left_chainbar) = -127; // lifts the chainbar up
	motor(right_chainbar) = -127; // lifts the chainbar up
	wait1Msec(1600); // lift it for 1.4 seconds
	motor(left_chainbar) = 0; // reset chainbar motor
	motor(right_chainbar) = 0; // reset chain bar motor
	wait1Msec(100); // robot rests for .1 second before further action
	motor(mobile_goal_intake) = -127;	// put mobile goal intake down
	wait1Msec(1000); // lasting period
	motor(mobile_goal_intake) = 0; // stop


	SensorValue[encoder_left] = 0; // (reset encoder)
	SensorValue[encoder_right] = 0; // (reset encoder)

	while(abs(SensorValue[encoder_right]) < 1250)	// drive forward to mobile goal
	{
		motor[drive_right] = 127; //motor power
		motor[drive_left] = 127; //motor power
	}
	motor[drive_right] = 0; // stop
	motor[drive_left] = 0; // stop


	motor(mobile_goal_intake) = 127;	// pick up mobile goal
	wait1Msec(1800); //lasting period
	motor(mobile_goal_intake) = 0; //stop
	motor[lift_bottom_left]= -127;
	motor[lift_bottom_right] = -127;
	motor[lift_top_left] = 127;
	motor[lift_top_right] = 127;
	wait1Msec(1000);
	motor[lift_bottom_left]= 0;
	motor[lift_bottom_right] = 0;
	motor[lift_top_left] = 0;
	motor[lift_top_right] = 0;
	motor[claw] = 127;
	wait1Msec(2000);
	motor[left_chainbar] = 127;
	motor[right_chainbar] = 127;
	wait1Msec(1500);
	motor[left_chainbar] = 0;
	motor[right_chainbar] = 0;
	motor[lift_bottom_left]= -127;
	motor[lift_bottom_right] = 127;
	motor[lift_top_left] = -127;
	motor[lift_top_right] = 127;
	wait1Msec(1000);
	motor[claw] = 0;
	motor[claw] = -127;
	wait1Msec(200);
	motor[claw] = 0;
	motor[right_chainbar] = -127;
	motor[left_chainbar] = -127;
	wait1Msec(1600);
	motor[right_chainbar] = 0;
	motor[left_chainbar] = 0;



	SensorValue[encoder_right] = 0; // (reset encoder)
	SensorValue[encoder_left] = 0; // (reset encoder)

	while(abs(SensorValue[encoder_right]) < 300 && abs(SensorValue[encoder_left]) < 300)	//
	{
		motor[drive_left] = -127; // motor power
		motor[drive_right] = -127; // motor power
	}
	motor[drive_left] = 0; // stop
	motor[drive_right] = 0; // stop


	SensorValue[encoder_right] = 0; //(reset encoder)
	SensorValue[encoder_left] = 0; //(reset encoder)

	while(abs(SensorValue[encoder_right]) < 1420)	// turn around after mobile goal was picked up
	{
		motor[drive_right] = -127; // motor power
	}
	motor[drive_right] = 0; // stop


	SensorValue[encoder_right] = 0; // (reset encoder)
	SensorValue[encoder_left] = 0; // (reset encoder)

	while(abs(SensorValue[encoder_right]) < 600&& abs(SensorValue[encoder_left]) < 600)	// drives to five-point zone
	{
		motor[drive_left] = 127; // motor power
		motor[drive_right] = 127; // motor power
	}
	motor[drive_left] = 0; // stop
	motor[drive_right] = 0; // stop


	motor[mobile_goal_intake] = -127; // put mobile goal intake down
	wait1Msec(1000); // lasting period
	motor[mobile_goal_intake] = 0; // stop


	motor[drive_left] = -60; // drive back from the mobile goal
	motor[drive_right] = -60; // drive back from the mobile goal
	wait1Msec(300); // lasting period
	motor[drive_left] = 0; // stop
	motor[drive_right] = 0; // stop


	motor[mobile_goal_intake] = 127;
	wait1Msec(700); // lasting period
	motor[mobile_goal_intake] = 0; // stop


	SensorValue[encoder_right] = 0; // (reset encoder)
	SensorValue[encoder_left] = 0; // (reset encoder)

	while(abs(SensorValue[encoder_right]) < 300) // drive right turns 25 degrees right
	{
		motor[drive_right] = -127; // motor power
	}
	motor[drive_right] = 0; // stop


	while(abs(SensorValue[encoder_right]) < 500 && abs(SensorValue[encoder_left]) < 500) // drives away from five-point zone
	{
		motor[drive_right] = -127; // motor power
		motor[drive_left] = -127; // motor power
	}
	motor[drive_right] = 0; // stop
	motor[drive_left] = 0; // stop
}	// end autonomous



task usercontrol()	// controller setup
{
	while (true)
	{
		motor[drive_left] = vexRT[Ch3]; // left drive, left joystick, main controller
		motor[drive_right] = vexRT[Ch2]; // right drive, right joystick, main controller

		motor[lift_top_left] = vexRT[Ch3Xmtr2]; // lift, left joystick, partner controller
		motor[lift_bottom_left] = vexRT[Ch3Xmtr2]; // lift, left joystick, partner controller
		motor[lift_top_right] = vexRT[Ch3Xmtr2]; // lift, left joystick, partner controller
		motor[lift_bottom_right] = vexRT[Ch3Xmtr2]; // lift, left joystick, partner controller

		if(vexRT[Btn6UXmtr2]==1) // button 6U, partner controller
		{
			motor[claw] = 127; // motor power
		}
		else if(vexRT[Btn6DXmtr2]==1) // button 6D, partner controller
		{
			motor[claw] = -127; // motor power
		}
		else // if buttons are not pressed
		{
			motor[claw] = 0; // motor power
		}

		if(vexRT[Btn6U]==1) // button 6U, main controller
		{
			motor[mobile_goal_intake] = 127; // motor power
		}
		else if(vexRT[Btn6D]==1) // button 6D, main controller
		{
			motor[mobile_goal_intake] = -127; // motor power
		}
		else // if buttons are not pressed
		{
			motor[mobile_goal_intake] = 0; // motor power
		}

		if(vexRT[Btn5UXmtr2]==1) // button 5U, partner controller
		{
			motor[left_chainbar] = 127; // motor power
			motor[right_chainbar] = -127; // motor power
		}
		else if(vexRT[Btn5DXmtr2]==1) // button 5D, partner controller
		{
			motor[left_chainbar] = -127; // motor power
			motor[right_chainbar] = 127; // motor power
		}
		else // if buttons are not pressed
		{
			motor[left_chainbar] = 0; // motor power
			motor[right_chainbar] = 0; // motor power
		}


	}
}	// end program
