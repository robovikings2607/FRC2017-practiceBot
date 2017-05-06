package org.usfirst.frc.team2607.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.SpeedController;

public class Transmission implements SpeedController{
	
	CANTalon motor1 , motor2;
	PIDLogger logger;
	private String name;
	boolean pidEnabled , speedIsRPM= false , highGear = true;
 
	public Transmission(int channelA , int channelB , String name){
		motor1 = new CANTalon(channelA);
		motor2 = new CANTalon(channelB);
		this.name = name;
		
		motor2.changeControlMode(CANTalon.TalonControlMode.Follower);
		motor2.set(motor1.getDeviceID());
		motor2.enableBrakeMode(true);
		
		motor1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		motor1.configEncoderCodesPerRev(1024);
		motor1.reverseSensor(false);
		motor1.configNominalOutputVoltage(0.0, 0.0);
		motor1.configPeakOutputVoltage(12.0, -12.0);
		motor1.setProfile(0);
		
		/*
		 * 1024 nativeClicks / 1 encoderRotations
		 * 20 encoderRotations / 9 wheelRotations
		 * 4" wheel Diameter
		 * (1024/1) * (20/9) = 2275.556 nativeClicks / 1 wheelRotations
		 * 2275.556 nativeClicks / 12.566 inches (or 1.047 feet)
		 * 2173 nativeClicks / 1 foot
		 */
		
		setHighGearGains();
		
		
		logger = new PIDLogger(motor1, name);
		logger.start();
		
		pidEnabled = false;
		
	}
	
	public void setLowGearGains() {
		if (name.equalsIgnoreCase("Right Transmission")) {
			double Kp = 0.0;
			motor1.setF((1023.00 / 3000.0) * 1.045); // set to (1023 / nativeVelocity)
			motor1.setP(Kp);
			motor1.setI(0);
			motor1.setD(0.0);
		} else {
			double Kp = 0.0;
			motor1.setF((1023.0 / 2848.00) * 1.02); // set to (1023 / nativeVelocity)
			motor1.setP(Kp);
			motor1.setI(0);
			motor1.setD(0.0);
		}
	}
	
	public void setHighGearGains() {
		if(name.equalsIgnoreCase("Right Transmission")) {
			double Kp = 0.0; 
			motor1.setF((1023.0 / 6850.0) * 1.14);
			motor1.setP(Kp);
			motor1.setI(0);
			motor1.setD(0);
		} else {	
			double Kp = 0.0;  
			motor1.setF(1023.0 / 5582.0);
			motor1.setP(Kp);					
			motor1.setI(0);
			motor1.setD(0);
		}		
	}

	
	
	
	public void setMotionProfileLowGearGains() {
		if(name.equalsIgnoreCase("Right Transmission")) {
			double Kp = 102.3 / 170.0; //10.1 // 380.0 / 300.0
			motor1.setF((1023.00 / 3000.0) * 0.96); // set to (1023 / nativeVelocity) 1.045
			motor1.setP(Kp);
			motor1.setI(0);
			motor1.setD(Kp * 10.0);
		} else {	
			double Kp = 102.3 / 170.0;  //44.8 // 380.0 / 300.0
			motor1.setF((1023.0 / 2848.00) * 1.0); // set to (1023 / nativeVelocity)
			motor1.setP(Kp);					// start with 10% of error (native units)
			motor1.setI(0);
			motor1.setD(Kp * 10.0);
		}
	}

	public void setMotionProfileHighGearGains() {
		if(name.equalsIgnoreCase("Right Transmission")) {
			double Kp = 0; //60.0 / 200.0; 
			motor1.setF((1023.0 / 6850.0) * 1.14);
			motor1.setP(Kp);
			motor1.setI(0);
			motor1.setD(0);
		} else {	
			double Kp = 0; //23.0 / 100.0;  
			motor1.setF(1023.0 / 5582.0);
			motor1.setP(Kp);					// start with 10% of error (native units)
			motor1.setI(0);
			motor1.setD(0);
		}
	}
	
	public void setHighGear(boolean hg , boolean following) {
		if (highGear != hg) {
			highGear = hg;
			if (highGear) {
				if (following) {
					setMotionProfileHighGearGains();
				} else {
					setHighGearGains();	
				}
			} else { 
				if (following) {
					setMotionProfileLowGearGains();
				} else {
					setLowGearGains();
				}
			}
		}
	}
	
	@Override
	public void pidWrite(double output) {
		
	}
	
	@Override
	public double get() {
		return motor1.get();
	}
	
	public void enablePID( boolean enableLogging , boolean expectRPM) {
		motor1.changeControlMode(TalonControlMode.Speed);
		speedIsRPM = expectRPM;
		logger.enableLogging(enableLogging);
		pidEnabled = true;
	}
	
	public void enableVoltage() {
		motor1.changeControlMode(TalonControlMode.Voltage);
		logger.enableLogging(true);
	}
	
	public void disablePID() {
		motor1.changeControlMode(TalonControlMode.PercentVbus);
		logger.enableLogging(false);
		pidEnabled = false;
	}
	
	public void resetEncoder() {
		motor1.reset();
	}
	public double getDistance() {
		return motor1.getEncPosition();
	}
	
	public double getRate() {
		return motor1.getEncVelocity();
	}
	
	public double getError() {
		return motor1.getClosedLoopError();
	}
	
	@Override
	public void set(double speed) {
		// TODO Auto-generated method stub
		if (!speedIsRPM) { 
			if (highGear) speed = speed * 817.0; //lowest max rpm between the transmissions when in high gear
			else speed = speed * 416.75; //in low gear
		}
		motor1.set (speed);
		logger.updSetpoint(speed);
	}
	
	@Override
	public void setInverted(boolean isInverted) {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public boolean getInverted() {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	public void disable() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		
	}

	public CANTalon getMasterSRX() {
		return motor1;
	}
	
	public void enableMotionProfileMode(boolean enableLogging) {
		motor1.changeControlMode(TalonControlMode.MotionProfile);
		logger.enableLogging(enableLogging);
	}
	
	public String getName() {
		// TODO Auto-generated method stub
		return name;
	}
}