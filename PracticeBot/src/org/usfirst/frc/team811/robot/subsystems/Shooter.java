package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	SpeedController shootLeft = RobotMap.shootLeft;
	SpeedController shootRight = RobotMap.shootRight;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void testTurn() {
    	double leftSpeed = SmartDashboard.getNumber("shoot left speed");
    	double rightSpeed = SmartDashboard.getNumber("shoot right speed");
    	
    	shootLeft.set(leftSpeed);
    	shootRight.set(-rightSpeed);
    	
    	SmartDashboard.putNumber("speed being set left", shootLeft.get());
    	SmartDashboard.putNumber("speed being set right", shootRight.get());
    }
    
    public void stopShoot() {
    	shootLeft.set(0);
    	shootRight.set(0);
    }
}

