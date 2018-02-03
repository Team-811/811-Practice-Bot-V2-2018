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
	private final int LIDAR_ADDR = 0x62;	
	private static byte[] readData;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	private final int LIDAR_STATUS_REGISTER = 0x01;
	
	private int distance = 0;  // in cm?
	
	//private byte[] distance = new byte[2];
		
	public Lidar() {
		i2c = new I2C(Port.kOnboard, LIDAR_ADDR);
		//task = new LIDARUpdater();
		updateTimer = new java.util.Timer();
		//readData = new byte[1];
		//readData[0] = (byte)0x8f;

	}
		
		// Distance in cm
	public int getDistance() {
		return distance; //(int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
	}
 
	public double pidGet() {
		return getDistance();
	}
		
		// Start 10Hz polling
	public void start() {
		updateTimer.scheduleAtFixedRate(new LIDARUpdater(), 0, 1000);
	}
		
		// Start polling for period in milliseconds
	public void start(int period) {
		updateTimer.scheduleAtFixedRate(new LIDARUpdater(), 0, period);
	}
		
	public void stop() {
		updateTimer.cancel();
		updateTimer = new java.util.Timer();
	}
		
	
	// Update distance variable
	public void update() {
		
//		distance[0] = (byte)0x0;
//		distance[1] = (byte)0x4;
//		i2c.writeBulk(distance);	
//		
//		
//			boolean stat = true;
//			
//			while (stat)
//			{
//				readData[0] = (byte)0x01;
//				i2c.writeBulk(readData);
//				i2c.readOnly(readData, 1);
//				int x = readData[0] & 0x1;
//				if (x==0){
//					stat = false;
//				}
//			}
//			readData[0] = (byte) 0x8f;
//			i2c.writeBulk(readData);
//			if (!(i2c.readOnly( distance,2))) {//(LIDAR_DISTANCE_REGISTER, 2, distance))) {
//															// Read in measurement
//				//SmartDashboard.putString("lidar status", "read returned true");
//			}
//			
//			Timer.delay(0.01); // Delay to prevent over polling
			
		
		
		
		
//		//i2c.write(LIDAR_CONFIG_REGISTER, 0x04);
//		i2c.writeBulk(distance, 0x04);
//		Timer.delay(0.04);
//		//i2c.readOnly(LIDAR_DISTANCE_REGISTER, 2, distance);
//		i2c.readOnly(distance, 2);
//		Timer.delay(0.005);
				 
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
	
	public static void scanForDevice() {
		for (int address=1; address <= 126; address++) {
			I2C i2c = new I2C(Port.kOnboard, address);
			boolean aborted = i2c.addressOnly();
			if (!aborted){
				System.out.print("Address found at ");
				System.out.println(address);
				return;
			}
		}		
		System.out.println("No devices found.");
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