package org.usfirst.frc.team811.robot;

import edu.wpi.first.wpilibj.Relay;


public interface Config {
	
	//ports
	
	int DRIVE_RIGHT_PORT = 14;	
	int DRIVE_LEFT_PORT = 13;	
	
	
	int ULTRA_PORT = 0; //TODO
	
	//Drive
	double SPEED_SCALE = 0.85;
	double DRIVE_DISTANCE_PER_PULSE = 1/9.5;	
	double GYRO_DIFFERENCE_VALUE = 10; //TODO

	
	
	//controls
	//driver
	int FORWARD_DRIVE_AXIS = 1;	
	int TURN_DRIVE_AXIS = 4; 	
	int GYRO_RESET_BUTTON = 1;
	
	int SERVO_AXIS = 5;	//triggers
	int SERVO_PRESET_BUTTON = 2;
	
	
}