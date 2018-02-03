package org.usfirst.frc.team811.robot.commands;

import org.usfirst.frc.team811.robot.Robot;
import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class follow_trajectory extends Command {

    public follow_trajectory() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.configureFollower();
    	RobotMap.driveLeft.setSelectedSensorPosition(0, 0, 1);
    	RobotMap.driveRight.setSelectedSensorPosition(0, 0, 1);
    	RobotMap.ahrs.reset();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//long startTime=System.currentTimeMillis(); //Starts timer
    	//long elapsedTime=System.currentTimeMillis - startTime; //Gets timer
    	//if (elapsedTime==1){ //if the timer is equal to 1 milisecond
    	Robot.drive.followTrajectory();
    	//}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//Robot.drive.reset();
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
