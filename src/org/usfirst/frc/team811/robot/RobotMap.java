package org.usfirst.frc.team811.robot;

import com.kauailabs.navx.frc.AHRS;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap implements Config 
{
	//objects
	public static Joystick joystick1;
	public static Joystick joystick2;
	
    public static WPI_TalonSRX driveLeft;
	public static WPI_TalonSRX driveRight;
    public static Encoder driveEncoderLeft;
    public static Encoder driveEncoderRight;
    public static DifferentialDrive driveTrain;
    public static AnalogGyro driveGyro;
    public static PIDController pid;
    public static AHRS ahrs;
    public static PIDController turnController;
    public static AnalogInput ultra;
    

    
    public void init() 
    {
    	//initialize
    	joystick1 = new Joystick(1);
        joystick2 = new Joystick(2);

        ultra = new AnalogInput(ULTRA_PORT);
        ultra.setOversampleBits(4);
        ultra.setAverageBits(2);
            	
        driveLeft = new WPI_TalonSRX(FRONT_LEFT_PORT);
        driveRight = new WPI_TalonSRX(FRONT_RIGHT_PORT);
        driveTrain = new DifferentialDrive(driveLeft, driveRight);
        
        driveEncoderLeft = new Encoder(DRIVE_ENCODER_PORT_LEFT_1, DRIVE_ENCODER_PORT_LEFT_2);
        driveEncoderLeft.setReverseDirection(true);
        driveEncoderLeft.setDistancePerPulse(1/1513.0);

        driveEncoderRight = new Encoder(DRIVE_ENCODER_PORT_RIGHT_1, DRIVE_ENCODER_PORT_RIGHT_2);
        driveEncoderRight.setReverseDirection(false);
        
        ahrs = new AHRS(SPI.Port.kMXP);
        
       
    }
}
