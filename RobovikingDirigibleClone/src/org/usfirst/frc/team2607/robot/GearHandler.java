package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class GearHandler {
	
	Solenoid door , ramp , pickup;
	Talon rollersOfYsgramor;
	
	
	public GearHandler (){
		door = new Solenoid (Constants.pcmDeviceID , Constants.doorsSolenoid);
		ramp = new Solenoid (Constants.pcmDeviceID, Constants.rampSolenoid);
		pickup = new Solenoid (Constants.pcmDeviceID, Constants.pickupSolenoid);
		rollersOfYsgramor = new Talon(Constants.pickupRollersMotor);
	}
	public void setDoors(boolean t) {
		door.set(t);
	}
	
	public void setRamp(boolean wamp){
		ramp.set(wamp);
	}
	
	public void setPickup(boolean iceIceBaby) {
		pickup.set(iceIceBaby);
	}
	
	public void setRollers(double umLikeSeven) {
		rollersOfYsgramor.set(umLikeSeven);
	}
}

