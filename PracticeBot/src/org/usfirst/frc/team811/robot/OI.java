package org.usfirst.frc.team811.robot;

import org.usfirst.frc.team811.robot.commands.*;
import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Config 
{
	//creating buttons
	//public JoystickButton intake_in;
	//public JoystickButton intake_out;
	//public JoystickButton intake_stop;
	//public JoystickButton shoot;
	//public JoystickButton stopShoot;
	//public JoystickButton climber_up;
	//public JoystickButton climber_down;
	//public JoystickButton gyro_reset;
	//public JoystickButton servo_preset;
	//public JoystickButton winch_down;
	//public JoystickButton shoot_distance;
	
	public OI() 
	{
		//Button initialize
		/*
		intake_in = new JoystickButton(RobotMap.joystick2, INTAKE_IN_BUTTON);
		intake_in.whenPressed(new intake());
		intake_out = new JoystickButton(RobotMap.joystick2, INTAKE_OUT_BUTTON);
		intake_out.whenPressed(new intake_backspin());
		intake_stop = new JoystickButton(RobotMap.joystick2, INTAKE_STOP_BUTTON);
		intake_stop.whenPressed(new intake_stop());
		
		shoot = new JoystickButton(RobotMap.joystick2, SHOOTER_BUTTON);
		shoot.whenPressed(new shoot_aimshoot());
		//stopShoot = new JoystickButton(RobotMap.joystick2, 4);
		//stopShoot.whenPressed(new shoot_stop());
		shoot_distance = new JoystickButton(RobotMap.joystick2, SHOOTER_DISTANCE_BUTTON);
		shoot_distance.whenPressed(new shoot_di_aimshoot()); 
		
		climber_up = new JoystickButton(RobotMap.joystick2, CLIMBER_UP_BUTTON);
		climber_up.whenPressed(new climb_up());
		climber_down = new JoystickButton(RobotMap.joystick2, CLIMBER_DOWN_BUTTON);
		climber_down.whenPressed(new climb_down());
		
		winch_down = new JoystickButton(RobotMap.joystick2, WINCH_DOWN_BUTTON);
		winch_down.whenPressed(new winch_down());
		
		gyro_reset = new JoystickButton(RobotMap.joystick1, GYRO_RESET_BUTTON);
		gyro_reset.whenPressed(new gyro_reset());
		
		servo_preset = new JoystickButton(RobotMap.joystick1, SERVO_PRESET_BUTTON);
		servo_preset.whenPressed(new servo_preset()); */
		
		//SmartDashboard buttons
		/*SmartDashboard.putData("auto_breach", new auto_breach());
		SmartDashboard.putData("auto_breachshootFrontGoal", new auto_breachshootFrontGoal());
		SmartDashboard.putData("auto_breachshootLeftGoal", new auto_breachshootLeftGoal());
		SmartDashboard.putData("auto_breachshootRightGoal", new auto_breachshootRightGoal());
		SmartDashboard.putData("auto_reach", new auto_reach());
		
		SmartDashboard.putData("climb_down", new climb_down());
		SmartDashboard.putData("climb_up", new climb_up());
		
		SmartDashboard.putData("drive_auto(30)", new drive_auto(-164));
		SmartDashboard.putData("drive_turn_auto(30)", new drive_turn_auto(30));
		SmartDashboard.putData("drive_w_joysticks", new drive_w_joysticks());
		
		SmartDashboard.putData("gyro_reset", new gyro_reset());
		SmartDashboard.putData("drive encoder reset", new drive_encoder_reset());
		
		SmartDashboard.putData("intake", new intake());
		SmartDashboard.putData("intake back spin", new intake_backspin());
		
		SmartDashboard.putData("shoot_auto_aim", new shoot_aimshoot());
		SmartDashboard.putData("shoot_w_joysticks", new shoot_w_joysticks());
		SmartDashboard.putData("shoot", new shoot());
		SmartDashboard.putData("shoot stop", new shoot_stop());
		
		SmartDashboard.putData("servo up", new servo_up());
		SmartDashboard.putData("servo down", new servo_down());
		SmartDashboard.putData("servo preset", new servo_preset());
		*/
		SmartDashboard.putData("vision turn auto", new vision_turn_auto());
		SmartDashboard.putData("lidar run", new lidar_run());
		SmartDashboard.putData("shoot test", new shoot_test_turn());
		SmartDashboard.putData("vision strafe auto", new vision_strafe_auto());
		/*
		SmartDashboard.putData("winch down", new winch_down());
		
		SmartDashboard.putData("distance shoot", new shoot_distanceshoot());*/
	}
}