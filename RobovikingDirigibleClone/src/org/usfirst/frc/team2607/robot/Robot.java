package org.usfirst.frc.team2607.robot;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
/*
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
*/
import org.usfirst.frc.team2607.robot.auto.AutonomousEngine;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	//Shooter shooter;
	//Turret turret;
	Climber climber;
	public GearHandler gearHandler;
	public Transmission leftTrans , rightTrans;
	RobovikingStick driveController , opController;
	RobotDrive robotDrive;
	AutonomousEngine autoEngine;
	public Solenoid shifter;
	public Solenoid flap;
	Talon pickup;
	Thread Autothread = null;
	
	double targetSpeed = 0.0, rightVoltage = 0.0, leftVoltage = 0.0;
	double shooterTargetSpeed = 0.0;
	boolean enableShooterPID = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//shooter = new Shooter();
		//turret = new Turret();
		climber = new Climber(Constants.climberMotor);
		gearHandler = new GearHandler(Constants.gearSolenoid);
		leftTrans = new Transmission(Constants.leftMotorA , Constants.leftMotorB , "Left Transmission");
		rightTrans = new Transmission(Constants.rightMotorA , Constants.rightMotorB , "Right Transmission");
		shifter = new Solenoid(Constants.pcmDeviceID , Constants.shifterSolenoid);
		//flap = new Solenoid(Constants.pcmDeviceID , Constants.brakeSolenoid);
		pickup = new Talon(Constants.pickupMotor);
		robotDrive = new RobotDrive(leftTrans , rightTrans);
		robotDrive.setSafetyEnabled(false);
		driveController = new RobovikingStick(Constants.driverController);
		opController = new RobovikingStick(Constants.operatorController);
		autoEngine=new AutonomousEngine(this);
		autoEngine.loadSavedMode();
		
		SmartDashboard.putNumber("targetSpeed", targetSpeed);
		SmartDashboard.putNumber("rightVoltage", rightVoltage);
		SmartDashboard.putNumber("leftVoltage", leftVoltage);
		
		SmartDashboard.putNumber("shooterTargetSpeed", shooterTargetSpeed);
		SmartDashboard.putBoolean("enableShooterPID", enableShooterPID);

		
		// for tuning....webserver to view PID logs
    	Server server = new Server(5804);
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(5804);
        server.addConnector(connector);

        ResourceHandler resource_handler = new ResourceHandler();
        resource_handler.setDirectoriesListed(true);
        resource_handler.setWelcomeFiles(new String[]{ "/home/lvuser/index.html" });

        resource_handler.setResourceBase(".");
        
        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[] { resource_handler, new DefaultHandler() });
        server.setHandler(handlers);
        try {
			server.start();
			server.join();
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
	}
	
	int autonSwitch = 0;
	boolean autonModeRan = false;

	@Override
	public void autonomousInit() {
		switch(autonSwitch) {
		case 0: //Following a path
			Autothread = new Thread(autoEngine);
			Autothread.start();
			autonModeRan = true;
			break;
		case 1: //Run PID loop from SmartDash input
			shifter.set(true);
			leftTrans.setHighGear(false , true);
			rightTrans.setHighGear(false , true);
			leftTrans.enablePID(true, true);
			rightTrans.enablePID(true, true);
			break;
		case 2: //run based on voltage
			leftTrans.enableVoltage();
			rightTrans.enableVoltage();
			break;
		}
//		Autothread=new Thread(autoEngine);	
//		Autothread.start();
//		autonModeRan=true;
	
//		leftTrans.enablePID(true, true);
//		rightTrans.enablePID(true, true);

//		leftTrans.enableVoltage();
//		rightTrans.enableVoltage();
	}
	
	@Override
	public void autonomousPeriodic() {
		switch(autonSwitch) {
		case 0:
			//Do Nothing
			break;
		case 1:
			double speed = SmartDashboard.getNumber("targetSpeed" , 0.0);
			leftTrans.set(-speed);
			rightTrans.set(speed);
			break;
		case 2:
			shifter.set(true);
			rightVoltage = SmartDashboard.getNumber("rightVoltage",0.0);
			leftVoltage = SmartDashboard.getNumber("leftVoltage",0.0);
			leftTrans.set(leftVoltage);
			rightTrans.set(rightVoltage);
			SmartDashboard.putNumber("leftSpeed", leftTrans.getRate());
			SmartDashboard.putNumber("rightSpeed", rightTrans.getRate());
			break;
		}
		
		/*
		shifter.set(true);
		leftTrans.setHighGear(false);
		rightTrans.setHighGear(false);
		double speed = SmartDashboard.getNumber("targetSpeed",0.0);
		leftTrans.set(-speed);
		rightTrans.set(speed);
		*/
		
		/*
		shifter.set(true);
		rightVoltage = SmartDashboard.getNumber("rightVoltage",0.0);
		leftVoltage = SmartDashboard.getNumber("leftVoltage",0.0);
		leftTrans.set(leftVoltage);
		rightTrans.set(rightVoltage);
		SmartDashboard.putNumber("leftSpeed", leftTrans.getRate());
		SmartDashboard.putNumber("rightSpeed", rightTrans.getRate());
		*/
	}
	
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void disabledPeriodic() {
		
		leftTrans.disablePID();
		rightTrans.disablePID();
		if (autonModeRan) {
			autonModeRan=false;
					if (Autothread.isAlive()) {
						System.out.println("autoThread alive, interrupting");
						Autothread.interrupt();
					}else{
						System.out.println("autoThread not alive");
					}
		}
		if (driveController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart)) {
			autoEngine.selectMode();
		}
		
	}
	
	@Override
	public void teleopInit() {
		leftTrans.enablePID(true, false);
		rightTrans.enablePID(true, false);
		//shooter.disablePID();
		shifter.set(true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//TELEOP STUFF------------
		robotDrive.arcadeDrive(driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) , 
				driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
		
		if(opController.getPOV(0) == 0 )   {
			climber.runForward();
		} else if(opController.getPOV(0) == 180) {
			climber.runBackwards();
		} else {
			climber.stopMotor();
		}
		
		//itsTheCliiiiiiiiiiiiiiiiiiiiiiimb.lockInPlace(opController.getToggleButton(RobovikingStick.xBoxButtonY));
		
		if(opController.getRawButton(RobovikingStick.xBoxRightBumper)) {
			pickup.set(-0.5);
		} else if(opController.getRawButton(RobovikingStick.xBoxLeftBumper)) {
			pickup.set(0.5);
		} else {
			pickup.set(0.0);
		}
		/*
		SmartDashboard.putNumber("leftSpeed", leftTrans.getRate());
		SmartDashboard.putNumber("rightSpeed", rightTrans.getRate());
		
		SmartDashboard.putNumber("leftError", leftTrans.getError());
		SmartDashboard.putNumber("rightError", rightTrans.getError());
		*/
		
		//SHOOTER STUFF-A-ROO
		/*
		shooter.setShooterSpeed(SmartDashboard.getNumber("shooterTargetSpeed", 0.0));
		SmartDashboard.putNumber("shooterEncSpeed", shooter.getShooterEncSpeed());
		System.out.println(shooter.getInfo());
		shooter.load(opController.getTriggerPressed(RobovikingStick.xBoxRightTrigger));
		*/
		
		shifter.set(driveController.getToggleButton(RobovikingStick.xBoxButtonLeftStick));
		leftTrans.setHighGear(!shifter.get() , false);
		
		rightTrans.setHighGear(!shifter.get() , false);
		gearHandler.set(driveController.getRawButton(RobovikingStick.xBoxButtonA));
		climber.stop(driveController.getRawButton(RobovikingStick.xBoxButtonB));
		//climber.stop(opController.getRawButton(RobovikingStick.xBoxButtonB));
		
		
	}

	/**
	 * This function is called periodically during test mode
	 */
}

