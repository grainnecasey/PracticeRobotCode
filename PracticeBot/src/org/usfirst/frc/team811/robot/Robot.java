package org.usfirst.frc.team811.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//import org.opencv.core.Core;
import org.usfirst.frc.team811.robot.commands.*;
import org.usfirst.frc.team811.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	double[] cenY;
	//public static Climber climber;
	//public static PortcullisLifter lifter;
	public static Drive drive;
	//public static Intake intake;
	//public static Shooter shooter;
	public static Vision tracker;
	//public static ServoCam servocam;

	public static OI oi;
	public static RobotMap robotMap;
	//public static Drive drive;

	Command autonomousCommand;
	SendableChooser autoChooser;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() 
	{

		robotMap = new RobotMap();
		robotMap.init();

		//climber = new Climber();
		//lifter = new PortcullisLifter();
		drive = new Drive();
		//intake = new Intake();
		//shooter = new Shooter();
		tracker = new Vision();
		//servocam = new ServoCam();

		final NetworkTable grip = NetworkTable.getTable("grip");

		oi = new OI();
		// System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
/*
		autoChooser = new SendableChooser();
		autoChooser.addDefault("reach defense", new auto_reach());
		autoChooser.addObject("breach defense", new auto_breach());
		autoChooser.addObject("breach shoot front goal",
				new auto_breachshootFrontGoal());
		autoChooser.addObject("breach shoot left goal",
				new auto_breachshootLeftGoal());
		autoChooser.addObject("breach shoot right goal",
				new auto_breachshootRightGoal());
		SmartDashboard.putData("Auto Mode", autoChooser);
				*/
		 /*try 
		 {
		 new ProcessBuilder("/home/lvuser/grip").inheritIO().start();
		 } 
		 catch (IOException e) 
		 {
		 e.printStackTrace();
		 }*/
	}

	public void disabledPeriodic() 
	{
		Scheduler.getInstance().run();
	}

	public void autonomousInit() 
	{
		// schedule the autonomous command (example)
		autonomousCommand = (Command) autoChooser.getSelected();
		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();

	}

	public void teleopInit() 
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		SmartDashboard.putDouble("distance", 0);
	}

	/**
	 * This function is called when the disabled button is hit. You can use it
	 * to reset subsystems before shutting down.
	 */
	public void disabledInit() 
	{

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic()
	{
		double[] defaultValue = new double[1];
		defaultValue[0] = 0;
		//cenY = RobotMap.visionTable.getNumberArray("centerY", defaultValue);
		Scheduler.getInstance().run();
		/*
		SmartDashboard.putNumber("gyro value", RobotMap.ahrs.getYaw());
		SmartDashboard.putNumber("drive encoder distance",
				RobotMap.driveEncoder.getDistance());
		SmartDashboard.putNumber("drive encoder raw",
				RobotMap.driveEncoder.getRaw());
		SmartDashboard.putNumber("drive encoder get",
				RobotMap.driveEncoder.get());
		SmartDashboard.putBoolean("intake limit switch",
				RobotMap.intakeLimit.get());
		SmartDashboard.putNumber("shooter encoder rate",
				RobotMap.shooterEncoder.getRate());
		SmartDashboard.putBoolean("climber motor forward limit",
				RobotMap.climberTalon1.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("climber motor reverse limit",
				RobotMap.climberTalon1.isRevLimitSwitchClosed());
		SmartDashboard.putBoolean("climber winch forward limit",
				RobotMap.climberTalon2.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("climber winch reverse limit",
				RobotMap.climberTalon2.isRevLimitSwitchClosed());
		SmartDashboard.putBoolean("climber top limit", RobotMap.climbertopinput.get());
		SmartDashboard.putBoolean("climber bottom limit", RobotMap.climberbottominput.get());
		SmartDashboard.putNumber("servo position", RobotMap.servoCam.get());
		SmartDashboard.putNumber("vision tracking distance", cenY[0]);//got an index out of bounds here
		// System.out.println(RobotMap.intakeLimit.get());
		*/
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() 
	{
		LiveWindow.run();
	}
}
