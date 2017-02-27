package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Climber {
	//TODO define a Talon that will control the climber motor
	
	//TODO make a 'Climber' constructor that takes in parameters for the Talon's PWM channel
	//TODO instantiate the Talon using the parameter that's passed in
	
	//TODO make a method that spins the motor when called
	
	//TODO make a method that spins the motor in reverse when called

	Talon talonMotor;
	Solenoid climberBrake;
	int counter;
	public Climber(int pwmChannel) {
		talonMotor = new Talon(pwmChannel);
		climberBrake = new Solenoid(Constants.pcmDeviceID, Constants.brakeSolenoid);
		counter = 0;
	 }
	 
	public void runForward(){
	 climberBrake.set(true);
	 counter ++;
	 if(counter >= 20){ 
		 talonMotor.set(1.0);	
	 	}
	 }
	
	public void runBackwards(){
		climberBrake.set(true);
		counter++;
		if(counter >= 20){
			talonMotor.set(-1.0);
		}
	}
	
	public void stop(){
		talonMotor.set(0.0);
		counter = 0;
		climberBrake.set(false);
	}
	
	public void lockInPlace(){
		climberBrake.set(false);
	}
	
	public void unlockFromPlace(){
		climberBrake.set(true);
	}
}