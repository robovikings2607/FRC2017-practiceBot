package org.usfirst.frc.team2607.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Talon;

public class Shooter {
	
	CANTalon shooterMaster, shooterFollower;
	Talon loader;
	PIDLogger logger;
	double targetSpeed = 0.0;
	public Shooter() {
		shooterMaster = new CANTalon(Constants.shooterMotorMaster);
		shooterFollower = new CANTalon(Constants.shooterMotorFollower);
		
		loader= new Talon(Constants.loaderMotor);
		
		shooterFollower.changeControlMode(TalonControlMode.Follower);
		shooterFollower.set(Constants.shooterMotorMaster);
		
		shooterMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooterMaster.reverseSensor(false);
		shooterMaster.configNominalOutputVoltage(0.0, 0.0);
		shooterMaster.configPeakOutputVoltage(12.0, -12.0);
		shooterMaster.setProfile(0);
		
		/*
		double kP = 102.3 / 2040.0; //0.050147
		//shooterMotor.setF(1023.0 / 118200.0);		// set to (1023 / nativeVelocity)
		shooterMaster.setF(1023.0 / 28340.0); //Max RPM 4100.0 @ 11.6 V
		shooterMaster.setP((kP * 1.75)); //1.75		// start with 10% of error (native units)
		shooterMaster.setI(0);
		shooterMaster.setD(kP * 30.0); //30.0
		// k: * 2.25 and d: * 35.0 is too aggressive
		 */
		
		//Max RPM: 4150 @ EncSpeed: 28300 @ 11.7V
		shooterMaster.setF(1023.0 / 28300.0);
		shooterMaster.setP(0.0);
		shooterMaster.setI(0.0);
		shooterMaster.setD(0.0);
		
		logger = new PIDLogger(shooterMaster,"shooterWheel");
		logger.start();
	}
	
	public void enablePID() {
		shooterMaster.changeControlMode(TalonControlMode.Speed);
		logger.enableLogging(true);
	}
	
	public void disablePID() {
		shooterMaster.changeControlMode(TalonControlMode.PercentVbus);
		logger.enableLogging(false);
	}
	
	public void usePID(boolean use) {
		if(use) shooterMaster.changeControlMode(TalonControlMode.Speed);
		else shooterMaster.changeControlMode(TalonControlMode.PercentVbus);
		logger.enableLogging(use);
	}
	public void load(boolean switcher) {
		if(switcher) loader.set(0.425);
		else loader.set(0.0);
	}

	public void set(double speed) {
		shooterMaster.set(speed);
		targetSpeed = speed;
		//logger.updSetpoint(speed);
	}
	
	public double getShooterEncSpeed() {
		return shooterMaster.getEncVelocity();
	}
	
	public String getInfo() {
		return "SHOOTER: target: " + targetSpeed + "speed: " + shooterMaster.getSpeed() + "enc: " + shooterMaster.getEncVelocity() 
		+ "outputV: " + shooterMaster.getOutputVoltage() + "Err: " + shooterMaster.getClosedLoopError();
	}
	
}