package org.usfirst.frc.team811.robot.commands;

import org.usfirst.frc.team811.robot.Robot;
import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class vision_strafe_auto extends Command {

    public vision_strafe_auto() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.tracker);
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.tracker.strafeTunePID();
    	Robot.tracker.gearStrafeCenter();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.tracker.setCamSource();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.tracker.strafeController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.tracker.strafeController.disable();
    	RobotMap.driveTrain.arcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.tracker.turnController.disable();
    	RobotMap.driveTrain.arcadeDrive(0, 0);
    }
}
