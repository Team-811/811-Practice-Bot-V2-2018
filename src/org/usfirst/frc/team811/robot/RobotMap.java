package org.usfirst.frc.team811.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

            	
        driveLeft = new WPI_TalonSRX(DRIVE_LEFT_PORT); 
        driveLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        driveLeft.setSensorPhase(false); /* keep sensor and motor in phase */
        driveLeft.configNeutralDeadband(0.01, 0);
        
        
        driveRight = new WPI_TalonSRX(DRIVE_RIGHT_PORT);     
        driveRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        driveRight.setSensorPhase(true); /* keep sensor and motor in phase */
        driveRight.configNeutralDeadband(0.01, 0);
        
        
        driveTrain = new DifferentialDrive(driveLeft, driveRight);
 
        
        ahrs = new AHRS(SPI.Port.kMXP);
        
       
    }
}
