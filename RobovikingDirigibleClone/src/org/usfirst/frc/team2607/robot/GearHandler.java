package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class GearHandler {
	
	//TODO add a Solenoid (pneumatic control switch) object
	Solenoid door;
	
	public GearHandler (int port){
		//TODO set the Solenoid to a "port number" parameter
		door = new Solenoid (Constants.pcmDeviceID , port);
	 
	}
	public void set(boolean t) {
		door.set(t);
	}
		
	public boolean get (){
		return door.get();
	}
}

