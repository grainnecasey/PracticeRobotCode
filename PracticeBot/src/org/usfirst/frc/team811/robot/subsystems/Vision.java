package org.usfirst.frc.team811.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import javafx.scene.image.Image;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.*;

//import org.opencv.core.*;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.videoio.VideoCapture;
import javax.imageio.ImageIO;



//import org.usfirst.frc.team811.robot.commands.imagetrack;
import org.usfirst.frc.team811.robot.Config;
import org.usfirst.frc.team811.robot.Robot;
import org.usfirst.frc.team811.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

public class Vision extends Subsystem implements Config, PIDOutput {
	private double[] cenX;

	private double[] area;
	private double[] height;
	private double[] width;
	private double[] cenY;
	private double[] defaultValue = new double[1];

	RobotDrive driveTrain = RobotMap.driveTrain;

	// the command from the PID controller
	public void pidWrite(double output) {
		SmartDashboard.putNumber("pid loop d", -output);
		driveTrain.arcadeDrive(0.0, -output);
	}
	
	//strafe command from PID controller
	public void strafePidWrite(double output) {
		SmartDashboard.putNumber("strafe pid output", -output);
		//driveTrain.mecanumDrive_Cartesian(-output, 0.0, 0.0, ahrs.getYaw());
		driveTrain.arcadeDrive(-output, 0);
	}

	AHRS ahrs = RobotMap.ahrs;

	public PIDController turnController = RobotMap.turnController; // RobotMap.turnController;
	public PIDController strafeController = RobotMap.strafeController;

	double rotateToAngleRate;
	double kTolerancePx = 2;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	@Override
	protected void initDefaultCommand() {
		turnController = new PIDController(kP, kI, kD, kF, ahrs,
			(PIDOutput) this);
		//SmartDashboard.putData((NamedSendable) RobotMap.turnController);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-.7, .7);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.setSetpoint(0.0);
		
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		
		strafeController = new PIDController(kP, kI, kD, kF, ahrs,
				(PIDOutput) this);
			//SmartDashboard.putData((NamedSendable) RobotMap.turnController);
			strafeController.setInputRange(-130.0f, 130.0f);
			strafeController.setOutputRange(-.5, .5);
			strafeController.setAbsoluteTolerance(kTolerancePx);
			strafeController.setContinuous(true);
			strafeController.setSetpoint(0.0);
			
			LiveWindow.addActuator("DriveSystem", "StrafeController", strafeController);
		
	}
	
	public void tunePID() {
		double P = SmartDashboard.getNumber("kP");
		double I = SmartDashboard.getNumber("kI");
		double D = SmartDashboard.getNumber("kD");
		double F = SmartDashboard.getNumber("kF");
		
		turnController.setPID(P, I, D, F);
		
	}
	
	public void strafeTunePID() {
		double P = SmartDashboard.getNumber("kP");
		double I = SmartDashboard.getNumber("kI");
		double D = SmartDashboard.getNumber("kD");
		double F = SmartDashboard.getNumber("kF");
		
		strafeController.setPID(P, I, D, F);
		
	}
	


	public int indexOfContour() {
		height = RobotMap.gearTable.getNumberArray("height", defaultValue);	//changed to gear one
		width = RobotMap.gearTable.getNumberArray("width", defaultValue);	//changed to gear one

		// need to worry about what happens when there is no target
		// what should be returned - maybe -1
		boolean targetLost = height.length == 0 || width.length == 0;
		if (targetLost) {
			SmartDashboard.putString("target Status", "no contours");
			return -1;
		}

		final double heightLimit = 4; // picked something high to start
		final double widthLimit = 20; 
		
		// is the target too small?
		targetLost = true;  // assume it is to start
		for (int i = 0; i < height.length - 1; i++) {
			if (height[i] > heightLimit && width[i] > widthLimit) {
				targetLost = false;
				SmartDashboard.putString("target Status", "target too small");
				break;
			}
		}
		if (targetLost) {
			return -1;
		}
		
		
		int index = -1;

		// 132 inches away from boiler: h = 9; w - 23; a = 140ish
		double errorH = 9; // average of heights at every 1/2 meter within
							// range
		double errorW = 30; // ^^ same with widths

		
		// finds which contour is most like the U shape should be
		for (int i = 0; i < height.length - 1; i++) {
			
			// what happens if the you pick up something that is small, 
			// if the height is 5 if will still pass this test but 5 is pretty small
			double eH = 9 - height[i];
			double eW = 30 - width[i];
			if (Math.abs(eH) < errorH && Math.abs(eW) < errorW) {
				errorH = Math.abs(eH);
				errorW = Math.abs(eW);
				index = i;
			}
		}

		if (index != -1) {
			SmartDashboard.putString("target Status", "target found");
			SmartDashboard.putNumber("height", height[index]);
			SmartDashboard.putNumber("width", width[index]);
			SmartDashboard.putNumber("error", turnController.getError());
		}

		return index;

	}

	public void gyroTurn() {
		// ensure that if something bad happens, everything stops
		// can use a try catch to stop the robot
		try {

			int indexOfTarget = indexOfContour();
			// if the index is -1 - the target was lost
			if (indexOfTarget == -1) {
				// what should the robot do?
				// for not stop and return
				turnController.setSetpoint(0);
				return;
			}

			// double currentRotationRate;

			defaultValue[0] = 0;
			cenX = RobotMap.visionTable.getNumberArray("centerX", defaultValue);

			

			double d = SmartDashboard.getDouble("distance");
			double i = (.274728886 * d + 42.40897141);	//42/45 * d; // inches displayed in
														// screen

			double r =Math.atan((1 / (2d))					//Math.atan(((i / 2) * d)	260/50
					* (1 - (cenX[indexOfTarget] / 130)));
			
			
			// angle needed to move in radians
			double x = -1 * Math.toDegrees(r); // angle needed to move in
			double dif= ahrs.getYaw() +r;
												// degrees
			
			
			// with a constant field of view (fov)
			double degreesPerPixel = 60.0/((double)framesizeX);
			double pixelsFromCenter = 130 - cenX[indexOfTarget];
			double errorInDegrees = degreesPerPixel * pixelsFromCenter;
			dif = ahrs.getYaw() - errorInDegrees;

			
			/*
			if (cenX[indexOfTarget] > 130) {
				dif = ahrs.getYaw() + r;
			} else {
				dif = ahrs.getYaw() - r;
			}*/
			
			SmartDashboard.putNumber("get setpoint", dif);
			//double dif = x;

			turnController.setSetpoint(dif);
			turnController.enable();

			

		} catch (RuntimeException ex) {
			// stop the PID loop and stop the robot
			turnController.disable();
			driveTrain.arcadeDrive(0.0, 0.0);
			throw ex;  // rethrow the exception - hopefully it gets displayed
		}
	}

	public void positionX() {
		defaultValue[0] = 0;
		cenX = RobotMap.visionTable.getNumberArray("centerX", defaultValue);
		area = RobotMap.visionTable.getNumberArray("area", defaultValue);
		boolean temp = true;
		int turn = 0; // 0 = center, 1 = right, 2 = left
		// while(temp)
		// {

		if (cenX[indexOfContour()] < 130 - framethres) {
			SmartDashboard.putString("Position X", "Left");
			driveTrain.arcadeDrive(0, 0.45);
			// turn = 1;
		} else if (cenX[indexOfContour()] > 130 + framethres) {
			SmartDashboard.putString("Position X", "Right");
			driveTrain.arcadeDrive(0, -0.45);
			// turn = 2;
		} else {
			SmartDashboard.putString("Position X", "Center");
			// if (turn == 1) {
			// driveTrain.arcadeDrive(0, -.5);
			// } else if (turn == 2) {
			// driveTrain.arcadeDrive(0, -.57);
			// }
			driveTrain.arcadeDrive(0, 0);
			temp = false;
			
		}

		/*
		 * 
		 * if (cenX.length == 3) {
		 * 
		 * 
		 * if (cenX[1] < framesizeX / 2 - framethres) {
		 * SmartDashboard.putString("Position X", "Left");
		 * driveTrain.arcadeDrive(0, -0.57); turn = 1; } else if (cenX[1] >
		 * framesizeX / 2 + framethres) { SmartDashboard.putString("Position X",
		 * "Right"); driveTrain.arcadeDrive(0, 0.57); turn = 2; } else {
		 * SmartDashboard.putString("Position X", "Center");
		 * driveTrain.arcadeDrive(0, 0); temp = false; } } else if (cenX.length
		 * == 2) { if (area[0] > area[1]) { if (cenX[0] < framesizeX / 2 -
		 * framethres) { SmartDashboard.putString("Position X", "Left");
		 * driveTrain.arcadeDrive(0, -0.57); turn = 1; } else if (cenX[0] >
		 * framesizeX / 2 + framethres) { SmartDashboard.putString("Position X",
		 * "Right"); driveTrain.arcadeDrive(0, 0.57); turn = 2; } else {
		 * SmartDashboard.putString("Position X", "Center");
		 * driveTrain.arcadeDrive(0, 0); temp = false; } } else { if (cenX[1] <
		 * framesizeX / 2 - framethres) { SmartDashboard.putString("Position X",
		 * "Left"); driveTrain.arcadeDrive(0, -0.57); turn = 1; } else if
		 * (cenX[1] > framesizeX / 2 + framethres) {
		 * SmartDashboard.putString("Position X", "Right");
		 * driveTrain.arcadeDrive(0, 0.57); turn = 2; } else {
		 * SmartDashboard.putString("Position X", "Center");
		 * driveTrain.arcadeDrive(0, 0); temp = false; } } } else if
		 * (cenX.length == 1) { if (cenX[0] < framesizeX / 2 - framethres) {
		 * SmartDashboard.putString("Position X", "Left");
		 * driveTrain.arcadeDrive(0, -0.57); turn = 1; } else if (cenX[0] >
		 * framesizeX / 2 + framethres) { SmartDashboard.putString("Position X",
		 * "Right"); driveTrain.arcadeDrive(0, 0.57); turn = 2; } else {
		 * SmartDashboard.putString("Position X", "Center");
		 * driveTrain.arcadeDrive(0, 0); temp = false; } } else { // if(turn ==
		 * 1) // driveTrain.arcadeDrive(0, 0.6); // else //
		 * driveTrain.arcadeDrive(0, -0.6); } // }
		 * 
		 * // Prob will change, if this doesn't work, just uncomment what is
		 * above /* boolean temp = true; while(temp) { if (cenX[0] < framesizeX
		 * / 2 - framethres) { SmartDashboard.putString("Position X", "Left");
		 * driveTrain.arcadeDrive(0, 0.3); } else if (cenX[0] > framesizeX / 2 +
		 * framethres) { SmartDashboard.putString("Position X", "Right");
		 * driveTrain.arcadeDrive(0, -0.3); } else {
		 * SmartDashboard.putString("Position X", "Center");
		 * driveTrain.arcadeDrive(0, 0); temp = false; } }
		 */
	}

	public boolean isCentered() {

		// defaultValue[0] = 0;
		cenX = RobotMap.visionTable.getNumberArray("centerX", defaultValue);
		// area = RobotMap.visionTable.getNumberArray("area", defaultValue);
		// boolean temp = true;
		// while(temp)
		// {
		int indexOfTarget = indexOfContour();
		// if the index is -1 - the target was lost
		if (indexOfTarget == -1) {
			SmartDashboard.putBoolean("is centered", false);
			return false;
		}

		
		double centerOfTarget = cenX[indexOfTarget];
		double centerOfCamera = framesizeX / 2;
		double error = centerOfCamera - centerOfTarget;

		boolean isCentered = Math.abs(error) <= framethres;
		//SmartDashboard.putNumber("get error", centerOfTarget);
		SmartDashboard.putBoolean("is centered", isCentered);

		return isCentered;
	}

	/*
	 * 
	 * if (cenX.length == 3) { if ((cenX[1] > framesizeX / 2 - framethres) &&
	 * (cenX[1] < framesizeX / 2 + framethres)) { return true; } } else if
	 * (cenX.length == 2) { if (area[0] > area[1]) { if (((cenX[0] > framesizeX
	 * / 2 - framethres) && (cenX[0] < framesizeX / 2 + framethres))) { return
	 * true; } } else { if (((cenX[1] > framesizeX / 2 - framethres) && (cenX[1]
	 * < framesizeX / 2 + framethres))) { return true; } } } else if
	 * (cenX.length == 1) { if (((cenX[0] > framesizeX / 2 - framethres) &&
	 * (cenX[0] < framesizeX / 2 + framethres))) { return true; } } else {
	 * return true; } return false;
	 */

	
	
	public void gearStrafeCenter() {
		
		int thresh = 5; 	//threshold of pixels 
		double dif;
		
		double rightTapePx = 198; 	//where right tape should be if centered
		double leftTapePx = 76; 	//where left tape should be if centered
		
		double cen;
		
		
		//input will be the number of pixels it has to move to whatever side
		
		try {

			int indexOfTarget = indexOfContour();
			// if the index is -1 - the target was lost
			if (indexOfTarget == -1) {
				// what should the robot do?
				// for not stop and return
				turnController.setSetpoint(0);
				return;
			}

			// double currentRotationRate;

			defaultValue[0] = 0;
			cenX = RobotMap.gearTable.getNumberArray("centerX", defaultValue);
			height = RobotMap.gearTable.getNumberArray("height", defaultValue);
			
			
			if (cenX.length < 2) {
				if (cenX[0] < 130) {
					dif = rightTapePx - cenX[0];
				} else {
					dif = leftTapePx - cenX[0];
				}
			} else {
				if (cenX[0] > cenX[1]) {
					cen = Math.abs(cenX[0] - cenX[1]) / 2 + cenX[1];
				} else {
					cen = Math.abs(cenX[0] - cenX[1]) / 2 + cenX[0];
				}
				
				//cen is the px of the center between left and right tapes in picture
				
				dif = 130 - cen;
			}
			
			SmartDashboard.putNumber("gear setpoint", dif);
			
			
			//then strafe so they are both equal distance from center
			
			strafeController.setSetpoint(dif);
			strafeController.enable();

		} catch (RuntimeException ex) {
			// stop the PID loop and stop the robot
			turnController.disable();
			driveTrain.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
			throw ex;  // rethrow the exception - hopefully it gets displayed
		}
		
		
	}
	
	
//	public double getDistance() {
//		/*
//		 * area = RobotMap.visionTable.getNumberArray("area", defaultValue);
//		 * double distance = area[0] * AREA_TO_DISTANCE; return distance;
//		 */
//		
//		
//		
//		cenY = RobotMap.visionTable.getNumberArray("centerY", defaultValue);
//
//		double height;
//
//		height = (framesizeY - cenY[indexOfContour()]);
//
//		SmartDashboard.putNumber("vision height", height);
//
//		double distance = (.02 * height) + 1.196; // needs work - with area??
//		return distance;
//		// return 0.0;
//	}
//
	/*
	 * private ScheduledExecutorService timer; private VideoCapture capture;
	 * private boolean cameraActive;
	 * 
	 * private Mat grabFrame() { Mat frame = new Mat(); // Mat frame = new
	 * Mat(frame1.getHeight(), frame1.getWidth(), // CvType.CV_8UC3); // byte[]
	 * pixels = //
	 * ((DataBufferByte)frame1.getRaster().getDataBuffer()).getData(); //
	 * frame.put(0, 0, pixels); if (this.capture.isOpened()) { try { Scalar
	 * hsv_min = new Scalar(83, 50, 50, 0); Scalar hsv_max = new Scalar(91, 255,
	 * 255, 0); // frame = this.doCanny(frame); Imgproc.cvtColor(frame, frame,
	 * Imgproc.COLOR_BGR2HSV); Core.inRange(frame, hsv_min, hsv_max, frame);
	 * Imgproc.GaussianBlur(frame, frame, new Size(3, 3), 3); } catch (Exception
	 * e) { System.err.println("Exception during the frame elaboration: " + e);
	 * } } return frame; }
	 * 
	 * private Mat doCanny(Mat frame) { Mat grayImage = new Mat(); Mat
	 * detectedEdges = new Mat(); Imgproc.cvtColor(frame, grayImage,
	 * Imgproc.COLOR_BGR2GRAY); Imgproc.blur(grayImage, detectedEdges, new
	 * Size(3, 3)); Imgproc.Canny(detectedEdges, detectedEdges, 10, 10 * 3, 3,
	 * false); Mat dest = new Mat(); Core.add(dest, Scalar.all(0), dest);
	 * frame.copyTo(dest, detectedEdges); return frame; }
	 * 
	 * /*private Image mat2Image(Mat frame) { MatOfByte buffer = new
	 * MatOfByte(); Imgcodecs.imencode(".png", frame, buffer); return new
	 * Image(new ByteArrayInputStream(buffer.toArray())); }
	 * 
	 * public void initDefaultCommand() { cameraActive = false;
	 * this.capture.open(0); if (this.capture.isOpened()) { this.cameraActive =
	 * true; Runnable frameGrabber = new Runnable() {
	 * 
	 * @Override public void run() { Mat imageToShow = grabFrame(); //
	 * currentFrame.setImage(imageToShow);
	 * 
	 * } }; this.timer = Executors.newSingleThreadScheduledExecutor();
	 * this.timer.scheduleAtFixedRate(frameGrabber, 0, 33,
	 * TimeUnit.MILLISECONDS); } else { //
	 * System.err.println("Cannot connect to camera."); } }
	 */

}
