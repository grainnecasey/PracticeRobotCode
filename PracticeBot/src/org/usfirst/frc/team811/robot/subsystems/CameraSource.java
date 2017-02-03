package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraSource implements PIDSource{
	
	private double[] cenX;
	private double[] defaultValue;
	
	double source;

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		double PIDReturnVal = source/260 -1;
		
		SmartDashboard.putNumber("cam source return val", PIDReturnVal);
		
		return source;
	}
	
	public void setSource(double currentCen) {
		source = currentCen;
		SmartDashboard.putNumber("cam source set source", source);
	}
	
	

}


/*#include "WPILib.h"

class CamSource : public PIDSource
{
public:
CamSource();

double PIDGet();
void SetSource(double);
private:
double source;

};

------------------------------------------------
My CamSource.cpp file is:
----------------------------------------
#include "CamSource.h"

CamSource :: CamSource()
{}

void CamSource :: SetSource (double sourcedum)
{
source = sourcedum;
}

double CamSource :: PIDGet()
{
return source;
}

// .h file
 #include <WPILib.h>
 class MyPIDSource : public PIDSource
 {
 public:

 double PIDGet();
 }

 // .cpp file
 #include "MyPIDSource.h"

 double MyPIDSource:IDGet()
 {
 double pidSourceValue;

 // calculate 'double' value that you want compared to the set point
 // feel free to format it the same as you expect with your set point
 pidSourceValue = ...

 // return value
 return pidSourceValue;
 }

*/