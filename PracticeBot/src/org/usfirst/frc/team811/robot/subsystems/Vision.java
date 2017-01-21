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

	AHRS ahrs = RobotMap.ahrs;

	public PIDController turnController = RobotMap.turnController; // RobotMap.turnController;

	double rotateToAngleRate;

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
		
	}
	
	public void tunePID() {
		double P = SmartDashboard.getNumber("kP");
		double I = SmartDashboard.getNumber("kI");
		double D = SmartDashboard.getNumber("kD");
		double F = SmartDashboard.getNumber("kF");
		
		turnController.setPID(P, I, D, F);
		
	}
	


	public int indexOfContour() {
		height = RobotMap.visionTable.getNumberArray("height", defaultValue);
		width = RobotMap.visionTable.getNumberArray("width", defaultValue);

		// need to worry about what happens when there is no target
		// what should be returned - maybe -1
		boolean targetLost = height.length == 0 || width.length == 0;
		if (targetLost) {
			SmartDashboard.putString("target Status", "target lost");
			return -1;
		}

		final double heightLimit = 35; // picked something high to start
		final double widthLimit = 25; 
		
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

		// 2m away: h: 42; w: 58; a: 400-500
		// 2.5m away: h: 36; w: 50; a: 300-380
		// 3. away: h: 35; w: 50; a: 200-300
		double errorH = 38; // average of heights at every 1/2 meter within
							// range
		double errorW = 51; // ^^ same with widths

		
		// finds which contour is most like the U shape should be
		for (int i = 0; i < height.length - 1; i++) {
			
			// what happens if the you pick up something that is small, 
			// if the height is 5 if will still pass this test but 5 is pretty small
			double eH = 38 - height[i];
			double eW = 51 - width[i];
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
		
		/*double kP = SmartDashboard.getNumber("kP");
		double kI = SmartDashboard.getNumber("kI");
		double kD = SmartDashboard.getNumber("kD");
		double kF = SmartDashboard.getNumber("kF");
		*/
		
		


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

			// needs distance from camera to U in inches (d = distance)
			// from distance, find width of camera view
			// then equate pixels to inches
			// then w = distance from center screen to center of U
			// then sin(x) = w/d

			double d = SmartDashboard.getDouble("distance");
			double i = (.274728886 * d + 42.40897141); // inches displayed in
														// screen

			double r = Math.atan((1 / 2d)
					* (1 - (cenX[indexOfTarget] / 130)));
			// angle needed to move in radians
			double x = -1 * Math.toDegrees(r); // angle needed to move in
												// degrees
			

			double dif = ahrs.getYaw() + x;
			
			SmartDashboard.putNumber("get setpoint", dif);
			//double dif = x;

			turnController.setSetpoint(dif);
			turnController.enable();

			// currentRotationRate = rotateToAngleRate;
			//
			// try {
			// /* Use the joystick X axis for lateral movement, */
			// /* Y axis for forward movement, and the current */
			// /* calculated rotation rate (or joystick Z axis), */
			// /* depending upon whether "rotate to angle" is active. */
			// driveTrain.mecanumDrive_Cartesian(0, 0,
			// currentRotationRate, ahrs.getAngle());
			// } catch( RuntimeException ex ) {
			// DriverStation.reportError("Error communicating with drive system:  "
			// + ex.getMessage(), true);
			// }

		} catch (RuntimeException ex) {
			// stop the PID loop and stop the robot
			turnController.disable();
			driveTrain.arcadeDrive(0.0, 0.0);
			throw ex;  // rethrow the exception - hopefully it gets displayed
		}
	}

	public void turnAuto() {

		height = RobotMap.visionTable.getNumberArray("height", defaultValue);
		width = RobotMap.visionTable.getNumberArray("width", defaultValue);

		int index = 0;
		// 2m away: h: 42; w: 58; a: 400-500
		// 2.5m away: h: 36; w: 50; a: 300-380
		// 3. away: h: 35; w: 50; a: 200-300
		double errorH = 38; // average of heights at every 1/2 meter within
							// range
		double errorW = 51;
		boolean changed = false;

		for (int i = 0; i < height.length - 1; i++) {
			double eH = 38 - height[i];
			double eW = 51 - width[i];
			if (Math.abs(eH) < errorH && Math.abs(eW) < errorW) {
				errorH = Math.abs(eH);
				errorW = Math.abs(eW);
				index = i;
				changed = true;
			}
		}

		if (changed) {
			positionX();
		} else {
			driveTrain.arcadeDrive(0, -0.6);
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
			return false;
		}

		
		double centerOfTarget = cenX[indexOfContour()];
		double centerOfCamera = framesizeX / 2;
		double error = centerOfTarget = centerOfCamera;

		boolean isCentered = Math.abs(error) <= framethres;
		//SmartDashboard.putNumber("get error", centerOfTarget);
		SmartDashboard.putBoolean("is centered", isCentered);

		return isCentered;

		// if (cenX[indexOfContour()]) {
		// if ((cenX[indexOfContour()] > framesizeX / 2 - framethres)
		// && (cenX[indexOfContour()] < framesizeX / 2 + framethres)) {
		// return true;
		// } else {
		// return false;
		// }
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

	public double getDistance() {
		/*
		 * area = RobotMap.visionTable.getNumberArray("area", defaultValue);
		 * double distance = area[0] * AREA_TO_DISTANCE; return distance;
		 */
		cenY = RobotMap.visionTable.getNumberArray("centerY", defaultValue);

		double height;

		height = (framesizeY - cenY[indexOfContour()]);

		SmartDashboard.putNumber("vision height", height);

		double distance = (.02 * height) + 1.196; // needs work - with area??
		return distance;
		// return 0.0;
	}

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
