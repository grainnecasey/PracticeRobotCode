package org.usfirst.frc.team811.robot.subsystems;


import java.awt.Robot;

import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.drive_w_joysticks;
import org.usfirst.frc.team811.robot.Config;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class Drive extends Subsystem implements Config {
	
    Joystick joy1 = RobotMap.joystick1;
    SpeedController frontright = RobotMap.drivefrontright;
    SpeedController frontleft = RobotMap.drivefrontleft;
    SpeedController backleft = RobotMap.drivebackleft;
    SpeedController backright = RobotMap.drivebackright;
    RobotDrive driveTrain = RobotMap.driveTrain;
    //Encoder driveEncoder = RobotMap.driveEncoder;
    //AnalogGyro driveGyro = RobotMap.driveGyro;
    //AHRS ahrs = RobotMap.ahrs;
    //PIDController turnController = RobotMap.turnController;
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void driveWithJoy() {
    	
    	double moveVal;
    	double turnVal;
    	
    	if ((joy1.getRawAxis(FORWARD_DRIVE_AXIS) < .2) && (joy1.getRawAxis(FORWARD_DRIVE_AXIS) > -.2)) { 
    		moveVal = 0;
    	} else {
    		moveVal = -joy1.getRawAxis(FORWARD_DRIVE_AXIS);
    	}
    	
    	if ((joy1.getRawAxis(TURN_DRIVE_AXIS) < .2) && (joy1.getRawAxis(TURN_DRIVE_AXIS) > -.2)) { 
    		turnVal = -.1;
    		
    	} else {
    		turnVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
    	}
    	
    	//driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);
    	driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * -1 * SPEED_SCALE);
    	
    	/* double leftVal = joy1.getRawAxis(FORWARD_DRIVE_AXIS); in case Joe wants tankdrive
    	 * double rightVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
    	 * driveRobotDrive41.tankDrive(leftVal, rightVal);
    	 */
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new drive_w_joysticks());
    }
    
    public void driveAuto(double driveDistance) {		//TODO drive distance!
    	/*double turnVal = ahrs.getAngle();
    	
    	//driveEncoder.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);
    	
    	while (driveEncoder.getDistance() <= driveDistance) {
    		driveTrain.tankDrive(-.3, -.4);
    		*/
    	}
    
    
    public void rotateToAngle(double setpoint) {
    	
    	//turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
       // turnController.setInputRange(-180.0f,  180.0f);
        //turnController.setOutputRange(-1.0, 1.0);
        //turnController.setAbsoluteTolerance(kToleranceDegrees);
        //turnController.setContinuous(true);
        
        double rotateToAngleRate = 0;
        
        //driveTrain.setSafetyEnabled(true);
        
            //turnController.setSetpoint(setpoint);
            //turnController.enable();
            double currentRotationRate;
            currentRotationRate = rotateToAngleRate;
            
            try {
                /* Use the joystick X axis for lateral movement,          */
                /* Y axis for forward movement, and the current           */
                /* calculated rotation rate (or joystick Z axis),         */
                /* depending upon whether "rotate to angle" is active.    */
                driveTrain.arcadeDrive(joy1.getRawAxis(FORWARD_DRIVE_AXIS), currentRotationRate);                 
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
            Timer.delay(0.005);		// wait for a motor update time
        
    }
        
    
    /*
    public void gyroReset() {
    	ahrs.reset();
    }
	

	@Override
	public void pidWrite(double output) {
		*/
		
	}



