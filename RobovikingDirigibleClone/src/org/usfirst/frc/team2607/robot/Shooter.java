package org.usfirst.frc.team2607.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class Shooter {
	
	CANTalon shooterMaster, shooterFollower, turnTable;
	Talon loader;
	PIDLogger logger;
	
	public Shooter() {
		shooterMaster = new CANTalon(Constants.shooterMotorMaster);
		shooterFollower = new CANTalon(Constants.shooterMotorFollower);
		
		loader= new Talon(Constants.loaderMotor);
		
		shooterFollower.changeControlMode(TalonControlMode.Follower);
		shooterFollower.set(Constants.shooterMotorMaster);
		
		shooterMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooterMaster.reverseSensor(false);
		shooterMaster.configNominalOutputVoltage(0.0, 0.0);
		shooterMaster.configPeakOutputVoltage(0, -12.0);
		shooterMaster.setProfile(0);
		//shooterMotor.setF(1023.0 / 118200.0);		// set to (1023 / nativeVelocity)
		shooterMaster.setF(0); //13070
		shooterMaster.setP(0);		// start with 10% of error (native units)
		shooterMaster.setI(0);
		shooterMaster.setD(0);
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
	
	public void load(boolean switcher) {
		if(switcher) loader.set(1.0);
		else loader.set(0.0);
	}

	public void setShooterSpeed(double speed) {
		shooterMaster.set(speed);
	}
	
	public double getShooterEncSpeed() {
		return shooterMaster.getEncVelocity();
	}
	
	public String getInfo() {
		return "speed: " + shooterMaster.getSpeed() + "enc: " + shooterMaster.getEncVelocity() 
		+ "outputV: " + shooterMaster.getOutputVoltage() + "Err: " + shooterMaster.getClosedLoopError();
	}
	
}