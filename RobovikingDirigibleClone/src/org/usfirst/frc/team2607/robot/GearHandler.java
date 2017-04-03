package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class GearHandler {
	
	//TODO add a Solenoid (pneumatic control switch) object
	Solenoid door , ramp;
	//Solenoid pickUp;
	
	
	public GearHandler (int port){
		//TODO set the Solenoid to a "port number" parameter
		door = new Solenoid (Constants.pcmDeviceID , port);
		ramp = new Solenoid (Constants.pcmDeviceID, Constants.rampSolenoid);
		//pickUp = new Solenoid (Constants.pcmDeviceID, Constants.pickUpSolenoid);
	}
	public void setDoors(boolean t) {
		door.set(t);
	}
	
	public void openRamp (boolean wamp){
		ramp.set(wamp);
	}
		
	public boolean get(){
		return door.get();
	}
}

