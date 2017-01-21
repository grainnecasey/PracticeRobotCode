package org.usfirst.frc.team811.robot;

/*import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
*/


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

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
	
	public static SpeedController drivefrontright;
    public static SpeedController drivefrontleft;
    public static SpeedController drivebackleft;
    public static SpeedController drivebackright;
    //public static Encoder driveEncoder;
    public static RobotDrive driveTrain;
    
    public static NetworkTable visionTable;
    
    public static AHRS ahrs;
    
    public static PIDController turnController;
   
 
    public void init() 
    {
    	//initialize
    	joystick1 = new Joystick(1);
        joystick2 = new Joystick(2);
    	
    	drivefrontright = new Talon(FRONT_RIGHT_PORT);
        drivefrontleft = new Talon(FRONT_LEFT_PORT);
        drivebackleft = new Talon(BACK_LEFT_PORT);
        drivebackright = new Talon(BACK_RIGHT_PORT);
        driveTrain = new RobotDrive(drivefrontleft, drivebackleft,
                drivefrontright, drivebackright);

        visionTable = NetworkTable.getTable("GRIP/811Contour");
        
        ahrs = new AHRS(SPI.Port.kMXP);
        
        //turnController = new PIDController(kP, kI, kD, kF, ahrs,
			//	(PIDOutput) this);
    }
}
