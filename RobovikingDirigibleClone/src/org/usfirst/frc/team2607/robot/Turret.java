package org.usfirst.frc.team2607.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Turret {
	CANTalon turret;
	PIDLogger logger;
	double targetSpeed = 0.0;
	
	public Turret() {
		turret = new CANTalon(Constants.turnTableMotor);
		
		turret.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		turret.reverseSensor(false);
		turret.configNominalOutputVoltage(0.0, 0.0);
		turret.configPeakOutputVoltage(12.0, 0.0);
		turret.setProfile(0);
	}
	
	public double getShooterEncPosition() {
		return turret.getEncPosition();
	}
	
	public String getInfo() {
		return "TURRET: enc: " + turret.getEncVelocity() + " outputV: " + turret.getOutputVoltage();
	}
}
