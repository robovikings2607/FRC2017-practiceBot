package org.usfirst.frc.team2607.robot;

public class Constants {

	public static final int leftMotorA = 12;
	public static final int leftMotorB = 14;
	public static final int rightMotorA = 11;
	public static final int rightMotorB = 13;
	
	public static final int pickupMotor = 0;
	public static final int climberMotor = 1;
	public static final int brakeSolenoid = 2;
	public static final int gearSolenoid = 1;
	public static final int shifterSolenoid = 0;
	public static final int pcmDeviceID = 0;
	
	public static final int driverController = 0;
	public static final int operatorController = 1;
	
	public static final double kWheelbaseWidth = 29.872 / 12.0;
	
	public static final double nativePerFoot = 3893.020921;  //2172.99549;
	
	public static final double feetToRotations(double feet) {
		return (feet * nativePerFoot) / 4096.0;
	}
	
	public static double feetPerSecondToRPM(double ftPerSec) {
		return ((ftPerSec * nativePerFoot * 60.0) / 4096.0);
	}

}
