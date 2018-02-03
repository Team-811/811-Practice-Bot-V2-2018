package org.usfirst.frc.team811.robot.subsystems;

import java.util.TimerTask;
import org.usfirst.frc.team811.robot.Config;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SensorBase;

public class Lidar extends Subsystem{

	private I2C i2c;
	private java.util.Timer updateTimer;
	private LIDARUpdater task;
	private final int LIDAR_ADDR = 0x62;	
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	private final int LIDAR_STATUS_REGISTER = 0x01;
	
	private int distance = 0;  // in cm?
		
	public Lidar() {
		i2c = new I2C(Port.kMXP, LIDAR_ADDR);
		task = new LIDARUpdater();
		updateTimer = new java.util.Timer();
	}
		
		// Distance in cm
	public int getDistance() {
		return distance;
	}
 
	public double pidGet() {
		return getDistance();
	}
		
		// Start 10Hz polling
	public void start() {
		updateTimer.scheduleAtFixedRate(task, 0, 1000);
	}
		
		// Start polling for period in milliseconds
	public void start(int period) {
		updateTimer.scheduleAtFixedRate(task, 0, period);
	}
		
	public void stop() {
		updateTimer.cancel();
	}
		
	
	// Update distance variable
	public void update() {
				 
		boolean success = i2c.write(LIDAR_CONFIG_REGISTER, 0x04);
		if (!success) {
			System.out.println("Failed to write reg - start read");
			return;
		}
		
		byte[] status = new byte[1];		
		do
		{
			success = i2c.read(LIDAR_STATUS_REGISTER, 1, status);
			if (!success) {
				System.out.println("Failed to read status");
				return;
			}
			System.out.print(status);
		} while (status[0] != 0);
		
		byte[] distanceBuffer = new byte[2];
		success = i2c.read(LIDAR_DISTANCE_REGISTER, 2, distanceBuffer); 
		if (!success) {
			System.out.println("Failed to read distance");
			return;
		}
		
		distance = (int)Integer.toUnsignedLong(distanceBuffer[0] << 8) + Byte.toUnsignedInt(distanceBuffer[1]);
		
		System.out.print(distanceBuffer[0]);
		System.out.print(", ");
		System.out.print(distanceBuffer[1]);
	}
		
		// Timer task to keep distance updated
	private class LIDARUpdater extends TimerTask {
		public void run() {
			update(); 
		}
	}

		@Override
	protected void initDefaultCommand() {
			// TODO Auto-generated method stub
			
		}
	}