package org.usfirst.frc.team811.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Gripper extends Subsystem {

	private static final int COMPRESSOR_PORT = 0; 
	private static final int OPEN_PORT = 0;
	private static final int CLOSE_PORT = 1;
	
	// NOTE: instantiating the solenoid will initialize the compressor class
	//   you only need the compressor if you need advanced control
	private DoubleSolenoid gripperPhematicControl;
	
	public Gripper() {
		gripperPhematicControl = new DoubleSolenoid(OPEN_PORT, CLOSE_PORT);
	}
	
	// TODO: need to verify the direction
	public void open()
	{
		gripperPhematicControl.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void close()
	{
		gripperPhematicControl.set(DoubleSolenoid.Value.kForward);
	}
	
	public void Neutral()
	{
		// Need to verify what this will do
		gripperPhematicControl.set(DoubleSolenoid.Value.kOff);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	
}