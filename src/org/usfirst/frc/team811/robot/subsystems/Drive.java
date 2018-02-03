package org.usfirst.frc.team811.robot.subsystems;


import java.awt.Robot;

import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.drive_w_joysticks;
import org.usfirst.frc.team811.robot.Config;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */

public class Drive extends Subsystem implements Config {
	
    Joystick joy1 = RobotMap.joystick1;
    WPI_TalonSRX driveLeft = RobotMap.driveLeft;
    WPI_TalonSRX driveRight = RobotMap.driveRight;
    DifferentialDrive driveTrain = RobotMap.driveTrain;
    EncoderFollower leftFollower;
    EncoderFollower rightFollower;
    Trajectory trajectory;
    		
    AHRS ahrs = RobotMap.ahrs;
    PIDController turnController = RobotMap.turnController;
    
    double max_velocity = 0.5;
    double max_acceleration = 0.5;
    double max_jerk = 60.0;
    double wheel_diameter = 0.15;
    double wheel_base_distance = 0.3759;
    int encoder_rotation = 1100;
    double kD = 0.0;
    double kP = 1;
    double acceleration_gain = 0.0;
    //TODO
    double absolute_max_velocity = 0.78;
    
    
    // Put methods for controlling this subsystem/
    // here. Call these from Commands.
    public void driveWithJoy() {
    	
    	double moveVal;
    	double turnVal;
    	
    	if ((joy1.getRawAxis(FORWARD_DRIVE_AXIS) < .2) && (joy1.getRawAxis(FORWARD_DRIVE_AXIS) > -.2)) { 
    		moveVal = 0;
    	} else {
    		moveVal = joy1.getRawAxis(FORWARD_DRIVE_AXIS);
    	}
    	
    	//if ((joy1.getRawAxis(TURN_DRIVE_AXIS) < .2) && (joy1.getRawAxis(TURN_DRIVE_AXIS) > -.2)) { 
    		//ahrs.reset();
    		//turnVal = ahrs.getYaw() * -.1;
    	//} else {
    		turnVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
    	//}
    	
    	driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);
    	
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new drive_w_joysticks());
    }
    
    
    
    
    
    
	public void generateTrajectory() {

		

		Waypoint[] points = new Waypoint[] {
				new Waypoint(0, 0, 0), 
				new Waypoint(5, 0, 0),																		
				//new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
				
				//new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		TankModifier modifier = new TankModifier(trajectory).modify(wheel_base_distance);
		leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier.getRightTrajectory());
		
		
		for (int i = 0; i < trajectory.length(); i++) {
		    Trajectory.Segment seg = trajectory.get(i);
		    
		    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
		        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
		            seg.acceleration, seg.jerk, seg.heading);
		 
		}
		
		
	}
	public void configureFollower() {
		
		driveLeft.setSelectedSensorPosition(0, 0, 1);
		driveRight.setSelectedSensorPosition(0, 0, 1);
		ahrs.reset();
		
		leftFollower.configureEncoder(driveLeft.getSelectedSensorPosition(0) , encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(driveRight.getSelectedSensorPosition(0), encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		
		
		
	}

	public void followTrajectory() {

		double l = leftFollower.calculate(driveLeft.getSelectedSensorPosition(0));
		double r = rightFollower.calculate(driveRight.getSelectedSensorPosition(0));

		double gyro_heading = ahrs.getYaw() * -1; // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

		driveTrain.tankDrive(l + turn  , r - turn);
		
		System.out.println(l + " " + r);
	}	
	
	public void reset() {
		driveTrain.tankDrive(0, 0);
	}

	 
}

