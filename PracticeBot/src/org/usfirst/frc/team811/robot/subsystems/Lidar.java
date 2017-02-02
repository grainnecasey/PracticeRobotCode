package org.usfirst.frc.team811.robot.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;

 


public class Lidar extends Subsystem{

	
private I2C i2c;

	
private static byte[] distance;


	private static byte[] readData;


	private java.util.Timer updater;


	private LIDARUpdater task;
	

	public boolean i2caddressonly;

	


	private final int LIDAR_ADDR = 0x62;


	private final int LIDAR_CONFIG_REGISTER = 0x00;


	private final int LIDAR_DISTANCE_REGISTER = 0x8f;

	
	public int count = 0;

	


	public Lidar() {

	
	i2c = new I2C(I2C.Port.kOnboard, LIDAR_ADDR);//TODO

		
	//	i2caddressonly = i2c.addressOnly();


		distance = new byte[2];

		readData = new byte[1];

		readData[0] = (byte)0x8f;


		task = new LIDARUpdater();


		updater = new java.util.Timer();
		
		distance[0] = (byte)0x0;
		distance[1] = (byte)0x0;
		i2c.writeBulk(distance);


	}

	


	// Distance in cm

	
public static int getDistance() {


		return (int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);


	}

 

	

public double pidGet() {


		return getDistance();


	}

	

	
// Start 10Hz polling

	public void start() {

		updater.scheduleAtFixedRate(task, 0, 20);
		SmartDashboard.putString("lidar status", "started");

	}

	

	// Start polling for period in milliseconds

	public void start(int period) {

		updater.scheduleAtFixedRate(task, 0, period);

	}

	

	public void stop() {

		updater.cancel();
		SmartDashboard.putString("lidar status", "canceled");

	}

	

	// Update distance variable

	public void update() {
	//	if(i2c.addressOnly())
	//		SmartDashboard.putString("lidar status", "address is true");
	//	else
	//		SmartDashboard.putString("lidar status", "address is false");


	//	if (!(i2c.write(LIDAR_CONFIG_REGISTER, 0x04))) {

	//		Timer.delay(0.04); // Delay for measurement to be taken
		//	SmartDashboard.putString("lidar status", "write returned true");
	
		//	i2c.writeBulk(readData);
			
			
			boolean stat = true;
			
			while (stat)
			{
				readData[0] = (byte)0x01;
				i2c.writeBulk(readData);
				i2c.readOnly(readData, 1);
				int x = readData[0] & 0x1;
				if (x==0){
					stat = false;
				}
			}
			readData[0] = (byte) 0x8f;
			i2c.writeBulk(readData);
			if (!(i2c.readOnly( distance,2))) {//(LIDAR_DISTANCE_REGISTER, 2, distance))) {
															// Read in measurement
				SmartDashboard.putString("lidar status", "read returned true");
			}
			
			Timer.delay(0.01); // Delay to prevent over polling
			
			
			
			SmartDashboard.putNumber("lidar count", count);


	}
	
	
/*
	private java.util.Timer updater;

	private TimerTask task;

	

	private final int LIDAR_ADDR = 0x62;

	private final int LIDAR_CONFIG_REGISTER = 0x00;

	private final int LIDAR_DISTANCE_REGISTER = 0x8f;

	

	public Lidar(Port port) {

		i2c = new I2C(Port.kOnboard, LIDAR_ADDR);//TODO

		distance = new byte[2];

		task = new LIDARUpdater();

		updater = new java.util.Timer();

	}

	

	// Distance in cm

	public static int getDistance() {

		return (int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);

	}

 

	public double pidGet() {

		return getDistance();

	}

	

	// Start 10Hz polling

	public void start() {

		updater.scheduleAtFixedRate(task, 0, 20);

	}

	

	// Start polling for period in milliseconds

	public void start(int period) {

		updater.scheduleAtFixedRate(task, 0, period);

	}

	

	public void stop() {

		updater.cancel();

	}

	

	// Update distance variable

	public void update() {

		i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement

		Timer.delay(0.04); // Delay for measurement to be taken

		i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement

		Timer.delay(0.01); // Delay to prevent over polling


	}

	*/

	// Timer task to keep distance updated
	
	private class LIDARUpdater extends TimerTask {

		public void run() {

			while(true) {

				update();

				if(getDistance() < 90 && getDistance() > 84){

					SmartDashboard.putBoolean("Correct distance from human feeder", true);

				}

				else{

					SmartDashboard.putBoolean("Correct distance from human feeder", false);

				}

				

				if(getDistance() < 80 && getDistance() > 70){

					SmartDashboard.putBoolean("Correct distance to stacks", true);

				}

				else{

					SmartDashboard.putBoolean("Correct distance to stacks", false);

				}

				SmartDashboard.putNumber("LIDAR distance Inches", (getDistance() / 2.54));

				try {

					Thread.sleep(10);

				} catch (InterruptedException e) {

					e.printStackTrace();

				}

			}
			
		}
		
	}



	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	}

