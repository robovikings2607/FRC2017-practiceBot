package org.usfirst.frc.team2607.robot;

import com.ctre.phoenix.MotorControl.SmartMotorController.FeedbackDevice;
import com.ctre.phoenix.MotorControl.SmartMotorController.TalonControlMode;
import com.ctre.phoenix.MotorControl.CAN.TalonSRX;

public class Turret {
	TalonSRX turret;
	PIDLogger logger;
	double targetPosition = 0.0;
	
	//60.0 RPM = desired cruise velocity
	//
	public Turret() {
		turret = new TalonSRX(Constants.turnTableMotor);
		
		turret.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		turret.reverseSensor(false);
		turret.configNominalOutputVoltage(0.0, 0.0);
		turret.configPeakOutputVoltage(6.0, -6.0);
		turret.setProfile(0);
		turret.setF(0);
		turret.setP(0);
		turret.setI(0);
		turret.setD(0);
		turret.setMotionMagicCruiseVelocity(0);
		turret.setMotionMagicAcceleration(0);
	}
	
	public double getShooterEncPosition() {
		return turret.getEncPosition();
	}
	
	public void useMagic(boolean useMagic) {
		if(useMagic){
			turret.changeControlMode(TalonControlMode.MotionMagic);
		} else {
			turret.changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	
	public void set(double in) {
		turret.set(in);
	}
	
	public double getEncPosition(){
		return turret.getEncPosition();
	}
	
	public String getInfo() {
		return "TURRET: target: " + targetPosition + "pos: " + getEncPosition() + " speed: " + turret.getSpeed() + " enc: " + turret.getEncVelocity() 
		+ " outputV: " + turret.getOutputVoltage() + " Err: " + turret.getClosedLoopError();
	}
}
