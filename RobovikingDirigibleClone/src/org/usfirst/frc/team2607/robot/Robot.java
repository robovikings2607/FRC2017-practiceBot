package org.usfirst.frc.team2607.robot;

import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;

import org.usfirst.frc.team2607.robot.auto.AutonomousEngine;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Climber climber;
	public GearHandler gearHandler;
	public Transmission leftTrans , rightTrans;
	RobovikingStick driveController , opController;
	public RobotDrive robotDrive;
	AutonomousEngine autoEngine;
	public Solenoid shifter;
	Thread Autothread = null;
	PDPLogger pdpLogger;
	//public AHRS gyro;
	
	public Shooter shooter;
	
	double targetSpeed = 0.0, rightVoltage = 0.0, leftVoltage = 0.0;
	double shooterTargetSpeed = 0.0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().startAutomaticCapture(0);
		//CameraServer.getInstance().startAutomaticCapture(1);
		
		climber = new Climber(Constants.climberMotor);
		gearHandler = new GearHandler();
		leftTrans = new Transmission(Constants.leftMotorA , Constants.leftMotorB , "Left Transmission");
		rightTrans = new Transmission(Constants.rightMotorA , Constants.rightMotorB , "Right Transmission");
		shifter = new Solenoid(Constants.pcmDeviceID , Constants.shifterSolenoid);
		robotDrive = new RobotDrive(leftTrans , rightTrans);
		robotDrive.setSafetyEnabled(false);
		driveController = new RobovikingStick(Constants.driverController);
		opController = new RobovikingStick(Constants.operatorController);
		//gyro = new AHRS(Port.kMXP);
		autoEngine=new AutonomousEngine(this);
		autoEngine.loadSavedMode();
		
		shooter = new Shooter();
		
		pdpLogger = new PDPLogger();
		pdpLogger.start();
		
		//gearHandler.setDoors(Constants.gearClosed);
		
		SmartDashboard.putNumber("targetSpeed", targetSpeed);
		SmartDashboard.putNumber("rightVoltage", rightVoltage);
		SmartDashboard.putNumber("leftVoltage", leftVoltage);
		SmartDashboard.putNumber("shooterTargetSpeed", shooterTargetSpeed);

		
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
	
	int autonSwitch = 0; //originally 0
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
			shifter.set(Constants.lowGear);
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
	public void disabledInit() {
		pdpLogger.disable();
	}
	
	@Override
	public void teleopInit() {
		leftTrans.enablePID(true, false);
		rightTrans.enablePID(true, false);
		shifter.set(Constants.highGear);
		pdpLogger.enable();
		shooter.usePID(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	int ick = 0;
	@Override
	public void teleopPeriodic() {
		
	//DRIVING
		if(driveController.getTriggerPressed(RobovikingStick.xBoxRightTrigger)) {
			//robotDrive.arcadeDrive(0.0 , calcTurn(SmartDashboard.getNumber("degToRotate", 0.0)));
		} else {
			robotDrive.arcadeDrive(driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) , 
					driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
			shifter.set(driveController.getToggleButton(RobovikingStick.xBoxButtonLeftStick));
			leftTrans.setHighGear(!shifter.get() , false);
			rightTrans.setHighGear(!shifter.get() , false);
			//if(driveController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart)) gyro.reset();
		}
		
	//CLIMBER
		if(opController.getRawButton(RobovikingStick.xBoxButtonB) || driveController.getRawButton(RobovikingStick.xBoxButtonB)) {
			climber.stop();
		} else {
			if(Math.abs(opController.getRawAxis(RobovikingStick.xBoxLeftStickY)) > 0.2 ) {
				climber.run(opController.getRawAxis(RobovikingStick.xBoxLeftStickY));
			} else {climber.stop();}
		}
		
	//GEARS
		gearHandler.setDoors(driveController.getRawButton(RobovikingStick.xBoxButtonA));
		gearHandler.setRamp(!opController.getRawButton(RobovikingStick.xBoxButtonY));
		gearHandler.setPickup(opController.getRawButton(RobovikingStick.xBoxButtonA));
		
		if(opController.getRawButton(RobovikingStick.xBoxRightBumper)) gearHandler.setRollers(1.0);
		else if(opController.getRawButton(RobovikingStick.xBoxLeftBumper)) gearHandler.setRollers(-0.5);
		else gearHandler.setRollers(0.0);
		
	//SHOOTER?
		if(opController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) 
			shooter.set(SmartDashboard.getNumber("shooterTargetSpeed", 0.0));
		else if(opController.getTriggerPressed(RobovikingStick.xBoxRightTrigger)) shooter.set(0.85);
		else shooter.set(0.0);
		if(opController.getRawButton(RobovikingStick.xBoxButtonX)) shooter.load(true);
		else shooter.load(false);
		
		/*
		SmartDashboard.putNumber("leftSpeed", leftTrans.getRate());
		SmartDashboard.putNumber("rightSpeed", rightTrans.getRate());
		
		SmartDashboard.putNumber("leftError", leftTrans.getError());
		SmartDashboard.putNumber("rightError", rightTrans.getError());
		*/
		
	//CONSOLE MESSAGES
		if(ick++ > 25) {
			//System.out.println("Yaw: " + gyro.getYaw());
			System.out.println(shooter.getInfo());
			ick = 0;
		}
	}
	
	public void setupAutonConfig() {
		gearHandler.setDoors(Constants.gearClosed);
		shifter.set(Constants.lowGear);
		leftTrans.setHighGear(false, true);
		rightTrans.setHighGear(false, true);
	}
	
	/*public double calcTurn(double degToTurn) {
		//long timeoutMilli = 3000;
		//long startTime = System.currentTimeMillis();
		
		double kP = 0.0657; //0.053
		double maxTurn = 0.7;
		double tolerance = 0.5;
				
		//double error = degToTurn - gyro.getYaw();
		//System.out.println("calcTurn error: " + error);
			
		double calcTurn = kP * error;
			if (error <= 0){
				calcTurn = Math.max(-maxTurn, calcTurn - .25);
			} else {
				calcTurn = Math.min(maxTurn, calcTurn + .25);
			}
			
		if (gyro.getYaw() > (degToTurn - tolerance) && gyro.getYaw() < (degToTurn + tolerance)){
			return 0.0;
		} else {
			//System.out.println("CommandedVoltage: " + calcTurn);
			return calcTurn;
		}
	}
*/
	/*public void rotateDeg(double target) {
		long startTime = System.currentTimeMillis();
		long deltaTime;
		//FOR USE IN AUTONOMOUS MODES
		int idek = 0;
		boolean keepZeroing = true;
		gyro.reset();
		while(keepZeroing) {
			deltaTime = System.currentTimeMillis() - startTime;
			robotDrive.arcadeDrive(0.0, calcTurn(target));
			if(calcTurn(target) == 0.0) idek++;
			if(idek > 20) keepZeroing = false;
			else if(deltaTime >= 5000){ System.out.println("TIMED OUT: rotation could not be completed"); keepZeroing = false;}
		}
		
	}*/
}

