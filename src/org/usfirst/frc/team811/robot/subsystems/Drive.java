package org.usfirst.frc.team811.robot.subsystems;


import java.awt.Robot;

import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.drive_w_joysticks;
import org.usfirst.frc.team811.robot.Config;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
    WPI_TalonSRX leftMotor = RobotMap.driveLeft;
    WPI_TalonSRX rightMotor = RobotMap.driveRight;
    DifferentialDrive driveTrain = RobotMap.driveTrain;
    Encoder driveEncoderLeft = RobotMap.driveEncoderLeft;
    Encoder driveEncoderRight = RobotMap.driveEncoderRight;
    Trajectory trajectory;
    Trajectory leftTrajectory;
    Trajectory rightTrajectory;
    		
    //AnalogGyro driveGyro = RobotMap.driveGyro;
    AHRS ahrs = RobotMap.ahrs;
    
    
    double max_velocity = 0.7;
    double max_acceleration = 0.8;
    double max_jerk = 60.0;
    double wheel_diameter = 0.219;
    double wheel_base_distance = 0.3683;
    int encoder_rotation = 1000;
    double kD = 0.0;
    double kP = 1.0;
    double acceleration_gain = 0.3;
    //TODO
    double absolute_max_velocity = 0.78;
    
    int timeoutsMs = 10;
    int minBufferPoints = 5;
    int totalCnt = 0;
    MotionProfileStatus status = new MotionProfileStatus();
    SetValueMotionProfile setValue = SetValueMotionProfile.Disable;
    
    int loopTimeout = -1;
    int state = 0;
    boolean bStart = false;
    
    // Put methods for controlling this subsystem/
    // here. Call these from Commands.
    public void driveWithJoy() {
    	
    	double moveVal;
    	double turnVal;
    	
    	if ((joy1.getRawAxis(FORWARD_DRIVE_AXIS) < .2) && (joy1.getRawAxis(FORWARD_DRIVE_AXIS) > -.2)) { 
    		moveVal = 0;
    	} else {
    		moveVal = -joy1.getRawAxis(FORWARD_DRIVE_AXIS);
    	}
    	
    		turnVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
    	
    	
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
																						
				new Waypoint(2, 0, 0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
				//new Waypoint(6, 0, 0),
				//new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.1, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		TankModifier modifier = new TankModifier(trajectory).modify(wheel_base_distance);
		leftTrajectory = modifier.getLeftTrajectory();
		rightTrajectory = modifier.getRightTrajectory();
		
//		
//		for (int i = 0; i < trajectory.length(); i++) {
//		    Trajectory.Segment seg = trajectory.get(i);
//		    
//		    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
//		        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//		            seg.acceleration, seg.jerk, seg.heading);
//		}
		}
    
	public void startFilling() {
		/* create an empty point */
		TrajectoryPoint leftPoint = new TrajectoryPoint();
		TrajectoryPoint rightPoint = new TrajectoryPoint();
		
		

		/* did we get an underrun condition since last time we checked ? */
		if (status.hasUnderrun) {
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			leftMotor.clearMotionProfileHasUnderrun(0);
			rightMotor.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		leftMotor.clearMotionProfileTrajectories();
		rightMotor.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		leftMotor.configMotionProfileTrajectoryPeriod(0, timeoutsMs);
		rightMotor.configMotionProfileTrajectoryPeriod(0, timeoutsMs);
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			Trajectory.Segment leftSeg = leftTrajectory.get(i);
			Trajectory.Segment rightSeg = rightTrajectory.get(i);
			
			/* for each point, fill our structure and pass it to API */
			leftPoint.position = convertToRotations(leftSeg.position) * encoder_rotation; //Convert Revolutions to Units
			leftPoint.velocity = convertToRotations(leftSeg.velocity) * encoder_rotation * 10; //Convert RPM to Units/100ms
			leftPoint.headingDeg = 0; /* future feature - not used in this example*/
			leftPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			leftPoint.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			leftPoint.timeDur = GetTrajectoryDuration((int)leftSeg.dt);
			leftPoint.zeroPos = false;
			
			rightPoint.position = convertToRotations(rightSeg.position) * encoder_rotation; //Convert Revolutions to Units
			rightPoint.velocity = convertToRotations(rightSeg.velocity) * encoder_rotation * 10; //Convert RPM to Units/100ms
			rightPoint.headingDeg = 0; /* future feature - not used in this example*/
			rightPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			rightPoint.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			rightPoint.timeDur = GetTrajectoryDuration((int)rightSeg.dt);
			rightPoint.zeroPos = false;
			
			if (i == 0) { 
				leftPoint.zeroPos = true;
				leftPoint.zeroPos = true;	/* set this to true on the first point */

				leftPoint.isLastPoint = false;
				leftPoint.isLastPoint = false;
			}
			if ((i + 1) == totalCnt) {
				leftPoint.isLastPoint = true;
				rightPoint.isLastPoint = true;			/* set this to true on the last point  */
			}
			leftMotor.pushMotionProfileTrajectory(leftPoint);
			rightMotor.pushMotionProfileTrajectory(rightPoint);
		
		}
	}
		
	
	public void configureFollower() {
		
		//First make sure any points in the talons are cleared
		leftMotor.clearMotionProfileTrajectories();
		rightMotor.clearMotionProfileTrajectories();
		
		leftMotor.changeMotionControlFramePeriod(5);
		rightMotor.changeMotionControlFramePeriod(5);
		
		
		
		
		//TODO
		leftMotor.config_kF(0,(absolute_max_velocity / (20 * Math.PI)) * encoder_rotation  , timeoutsMs);
		leftMotor.config_kP(0, kP, timeoutsMs);
		leftMotor.config_kI(0, 0, timeoutsMs);
		leftMotor.config_kD(0, kD, timeoutsMs);
		
		//TODO
		rightMotor.config_kF(0, (absolute_max_velocity / (20 * Math.PI)) * encoder_rotation, timeoutsMs);
		rightMotor.config_kP(0, kP, timeoutsMs);
		rightMotor.config_kI(0, 0.0, timeoutsMs);
		rightMotor.config_kD(0, kD, timeoutsMs);
		
		leftMotor.configMotionProfileTrajectoryPeriod(10, 0); //Our profile uses 10 ms timing
		rightMotor.configMotionProfileTrajectoryPeriod(10, 0); //Our profile uses 10 ms timing
		
		leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutsMs);
		rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutsMs);
		
	}

	public void followTrajectory() {
		
	
		/* Get the motion profile status every loop */
		leftMotor.getMotionProfileStatus(status);
		rightMotor.getMotionProfileStatus(status);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		
		if (loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				System.out.println("talon not present");
			} else {
				--loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (leftMotor.getControlMode() != ControlMode.MotionProfile || rightMotor.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			state = 0;
			loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (state) {
				case 0: /* wait for application to tell us to start an MP */
					if (bStart) {
						bStart = false;
	
						setValue = SetValueMotionProfile.Disable;
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						state = 1;
						loopTimeout = timeoutsMs;
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (status.btmBufferCnt > minBufferPoints) {
						/* start (once) the motion profile */
						setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						state = 2;
						loopTimeout = timeoutsMs;
 					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (status.isUnderrun == false) {
						loopTimeout = timeoutsMs;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (status.activePointValid && status.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						setValue = SetValueMotionProfile.Hold;
						state = 0;
						loopTimeout = -1;
						
					}
					break;
			}
		}

			/* Get the motion profile status every loop */
			leftMotor.getMotionProfileStatus(status);
			rightMotor.getMotionProfileStatus(status);
			
			leftMotor.set(ControlMode.MotionProfile, setValue.value);
			rightMotor.set(ControlMode.MotionProfile, setValue.value);
			

		
	}	
	
	
	public void reset() {
		leftMotor.set(ControlMode.PercentOutput, 0);
		leftMotor.set(ControlMode.PercentOutput, 0);
	}
		
	
		
	private double convertToRotations(double length) {
		double rotations = length / (wheel_diameter * Math.PI);
		return rotations;
	}
		
	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
	 
}

