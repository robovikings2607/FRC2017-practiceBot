package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.Robot;
import org.usfirst.frc.team2607.robot.RobovikingDriveTrainProfileDriver;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;

/**
 * @author Cerora
 *
 */
public class AutonomousManager {
	  
	Robot robot;
	public ArrayList<AutonomousMode> modes = new ArrayList<AutonomousMode>();

	AutonomousManager(Robot robot){
		this.robot = robot;
		
		modes.add(new DoNothingFailsafe());
		modes.add(new DoNothing());
		modes.add(new CrossBaseline(robot));
		//modes.add(new CenterPeg(robot));
		//modes.add(new LeftPeg(robot));
		//modes.add(new RightPeg(robot));
		//modes.add(new LeftPegAlt(robot));
		modes.add(new CenterPegAlt(robot));
	}
	
	public AutonomousMode getModeByName (String name){
		for (AutonomousMode m : modes){
			if (m.getName().equals(name))
				return m;
		}
		
		try {
			throw new Exception();
		} catch (Exception e) {
			System.err.println("Mode not found");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
	public AutonomousMode getModeByIndex (int index){
		try {
			return modes.get(index);
		} catch (IndexOutOfBoundsException e){
			System.err.println("Mode out of array bounds");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
		
	
	/*
	 * BEGIN AUTON MODE DECLARATIONS
	 * 
	 * You must add the mode to the array once you define its class
	 */
	

	public class DoNothing extends AutonomousMode {
		
		DoNothing(){}
		
		@Override
		public void run() {
			System.out.println("Explicitly told not to move");
		}

		@Override
		public String getName() {
			return "00-DoNothing";
		}
		
	}
	
	public class DoNothingFailsafe extends AutonomousMode {
		
		DoNothingFailsafe(){}

		@Override
		public void run() {
			System.out.println("This shouldn't be running - Mode 0 selected for some reason");
		}

		@Override
		public String getName() {
			return "E-DoNothingFailsafe";
		}
		
	}
	
	public class CrossBaseline extends AutonomousMode {
		Path path;
		CrossBaseline(Robot r) {
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 5.0;
			config.max_jerk= 25.0;
			config.max_vel =7.0;
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(14.0 , 0.0 , 0.0));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "CrossBaseline");
		}
		@Override
		public void run() {
			robot.shifter.set(Constants.lowGear);
			robot.leftTrans.setHighGear(false, true);
			robot.rightTrans.setHighGear(false, true);
			
			try{ Thread.sleep(250);} catch(Exception e) {System.out.println("Error waiting for shifters to shift...");}
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
			driver.followPath();
			try { 
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "01-CrossBaseline";
		}
	}
	
	public class CenterPeg extends AutonomousMode {
		Path path;
		
		CenterPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 25.0;
			config.max_vel = 5.0;
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(7.3 , 0.0 , 0.0));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "CenterPeg");
		}
		
		@Override
		public void run() {
			robot.setupAutonConfig();
			try{ 
				Thread.sleep(250);
				RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
				driver.followPathBACKWARDS();
				while (!driver.isDone()) Thread.sleep(20);
				//robot.gearHandler.setDoors(Constants.gearOpen);
				Thread.sleep(500);
				robot.leftTrans.set(-100);
				robot.rightTrans.set(100);
				Thread.sleep(300);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				//robot.gearHandler.setDoors(Constants.gearClosed);
				robot.shifter.set(Constants.highGear);
				robot.leftTrans.setHighGear(true, false);
				robot.rightTrans.setHighGear(true, false);
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "02-CenterPeg";
		}
	}

	public class BlueLeftPeg extends AutonomousMode {
		Path path;
		BlueLeftPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 30.0;
			config.max_vel = 5.0;
			
			double kX = 7.9;
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX - 2.4 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX , -3.7 , 4.8));
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "BlueLeftPeg");
		}
		@Override
		public void run() {
			robot.gearHandler.setDoors(Constants.gearClosed);
			robot.shifter.set(Constants.lowGear);
			robot.leftTrans.setHighGear(false, true);
			robot.rightTrans.setHighGear(false, true);
			
			try{ Thread.sleep(250);} catch(Exception e) {System.out.println("Error waiting for shifters to shift...");}
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
			driver.followPathBACKWARDS();
			try { 
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
				/*
				robot.leftTrans.set(60);
				robot.rightTrans.set(-60);
				Thread.sleep(20);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				*/
				robot.gearHandler.setDoors(Constants.gearOpen);
				Thread.sleep(750);
				robot.leftTrans.set(-60);
				robot.rightTrans.set(60);
				Thread.sleep(200);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				Thread.sleep(200);
				robot.gearHandler.setDoors(Constants.gearClosed);
				robot.shifter.set(Constants.highGear);
				robot.leftTrans.setHighGear(true, false);
				robot.rightTrans.setHighGear(true, false);
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "03-BlueLeftPeg";
		}
	}
	
	public class BlueRightPeg extends AutonomousMode {
		Path path;
		BlueRightPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 30.0;
			config.max_vel = 5.0;
			
			double kX = 8.0;
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX - 2.7 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX , 4.3 , 1.5));
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "BlueRightPeg");
		}
		@Override
		public void run() {
			robot.gearHandler.setDoors(Constants.gearClosed);
			robot.shifter.set(Constants.lowGear);
			robot.leftTrans.setHighGear(false, true);
			robot.rightTrans.setHighGear(false, true);
			
			try{ Thread.sleep(250);} catch(Exception e) {System.out.println("Error waiting for shifters to shift...");}
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
			driver.followPathBACKWARDS();
			try { 
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
				robot.gearHandler.setDoors(Constants.gearOpen);
				Thread.sleep(750);
				robot.leftTrans.set(-60);
				robot.rightTrans.set(60);
				Thread.sleep(150);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				Thread.sleep(200);
				robot.gearHandler.setDoors(Constants.gearClosed);
				robot.shifter.set(Constants.highGear);
				robot.leftTrans.setHighGear(true, false);
				robot.rightTrans.setHighGear(true, false);
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "04-BlueRightPeg";
		}
	}
	
	public class RedLeftPeg extends AutonomousMode {
		Path path;
		RedLeftPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 30.0;
			config.max_vel = 5.0;
			
			double kX = 8.6;
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX - 3.7 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX , -4.5 , 4.8));
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "RedLeftPeg");
		}
		@Override
		public void run() {
			robot.gearHandler.setDoors(Constants.gearClosed);
			robot.shifter.set(Constants.lowGear);
			robot.leftTrans.setHighGear(false, true);
			robot.rightTrans.setHighGear(false, true);
			
			try{ Thread.sleep(250);} catch(Exception e) {System.out.println("Error waiting for shifters to shift...");}
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
			driver.followPathBACKWARDS();
			try { 
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
				robot.gearHandler.setDoors(Constants.gearOpen);
				Thread.sleep(750);
				robot.leftTrans.set(-60);
				robot.rightTrans.set(60);
				Thread.sleep(150);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				Thread.sleep(200);
				robot.gearHandler.setDoors(Constants.gearClosed);
				robot.shifter.set(Constants.highGear);
				robot.leftTrans.setHighGear(true, false);
				robot.rightTrans.setHighGear(true, false);
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "05-RedLeftPeg";
		}
	}
	
	public class RedRightPeg extends AutonomousMode {
		Path path;
		RedRightPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 30.0;
			config.max_vel = 5.0;
			
			double kX = 7.9;
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX - 2.4 , 0.0 , 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(kX , 3.7 , 1.5));
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "RedRightPeg");
		}
		@Override
		public void run() {
			robot.gearHandler.setDoors(Constants.gearClosed);
			robot.shifter.set(Constants.lowGear);
			robot.leftTrans.setHighGear(false, true);
			robot.rightTrans.setHighGear(false, true);
			
			try{ Thread.sleep(250);} catch(Exception e) {System.out.println("Error waiting for shifters to shift...");}
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path);
			driver.followPathBACKWARDS();
			try { 
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
				/*
				robot.leftTrans.set(60);
				robot.rightTrans.set(-60);
				Thread.sleep(20);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				*/
				robot.gearHandler.setDoors(Constants.gearOpen);
				Thread.sleep(750);
				robot.leftTrans.set(-60);
				robot.rightTrans.set(60);
				Thread.sleep(200);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				Thread.sleep(200);
				robot.gearHandler.setDoors(Constants.gearClosed);
				robot.shifter.set(Constants.highGear);
				robot.leftTrans.setHighGear(true, false);
				robot.rightTrans.setHighGear(true, false);
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "06-RedRightPeg";
		}
	}
	
	/*	PALISADES AUTONOMOUS MEASUREMENTS
	 * 	<> alliance wall & human player station corner ( center of gear handler --> left peg guide rail)
	 * 		- drive straight 67in.
	 * 		- rotate 60 deg.
	 * 		- drive straight 60in.
	 * 
	 *  <> alliance wall & boiler corner (center of gear handler --> right peg guide rail)
	 *  	- drive straight 38in.
	 *  	- rotate -60 deg. 
	 *  	- drive straight 107in.
	 */
	
	//TODO Add a switch to invert the angles that we turn at, since the red alliance wall is not the same as the blue alliance wall
	public class LeftPeg extends AutonomousMode {
		private Path path_0 , path_1 , path_2;
		
		LeftPeg(Robot r) {
			super(r);
			
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 25.0;
			config.max_vel = 5.0;
			
			TrajectoryGenerator.Config config_alt =new TrajectoryGenerator.Config();
			config_alt.dt = 0.05;
			config_alt.max_acc = 6.0;
			config_alt.max_jerk= 25.0;
			config_alt.max_vel = 9.0;
			
			WaypointSequence waypoints_0 = new WaypointSequence(10);
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(6.9 , 0.0 , 0.0));
			path_0 = PathGenerator.makePath(waypoints_0, config, Constants.kWheelbaseWidth, "LeftPeg_0");
			
			WaypointSequence waypoints_1 = new WaypointSequence(10);
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(6.0, 0.0, 0.0));
			path_1 = PathGenerator.makePath(waypoints_1, config, Constants.kWheelbaseWidth, "LeftPeg_1");
			
			WaypointSequence waypoints_2 = new WaypointSequence(10);
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(25.0, 0.0, 0.0));
			path_2 = PathGenerator.makePath(waypoints_2, config_alt, Constants.kWheelbaseWidth, "LeftPeg_2");
		}
		
		@Override
		public void run() {
			robot.setupAutonConfig();
			try {
				Thread.sleep(250); //WAITING FOR SHIFTERS
				RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_0);
				driver.followPathBACKWARDS();
				while(!driver.isDone()) Thread.sleep(20);
				
				robot.rotateDeg(62.5);
				Thread.sleep(30);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
				driver.followPathBACKWARDS();
				while(!driver.isDone()) Thread.sleep(20);
				
				System.out.println("RELEASE GEAR NOW!");
				//TODO Release Gear
				robot.gearHandler.setDoors(Constants.gearOpen);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
				driver.followPath();
				while(!driver.isDone()) Thread.sleep(20);
				
				robot.rotateDeg(-62.5);
				Thread.sleep(30);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_2);
				driver.followPathBACKWARDS();
			} catch (Exception e) { System.out.println("ERROR: stopping autonomous"); }
		}
		
		

		@Override
		public String getName() {
			// TODO Auto-generated method stub
			return "03-LeftPeg";
		}
		
	}
	
	public class RightPeg extends AutonomousMode {
		private Path path_0 , path_1 , path_2;
		
		RightPeg(Robot r) {
			super(r);
			
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 25.0;
			config.max_vel = 5.0;
			
			TrajectoryGenerator.Config config_alt =new TrajectoryGenerator.Config();
			config_alt.dt = 0.05;
			config_alt.max_acc = 6.0;
			config_alt.max_jerk= 25.0;
			config_alt.max_vel = 9.0;
			
			WaypointSequence waypoints_0 = new WaypointSequence(10);
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(5.0 , 0.0 , 0.0));
			path_0 = PathGenerator.makePath(waypoints_0, config, Constants.kWheelbaseWidth, "RightPeg_0");
			
			WaypointSequence waypoints_1 = new WaypointSequence(10);
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(7.5, 0.0, 0.0));
			path_1 = PathGenerator.makePath(waypoints_1, config, Constants.kWheelbaseWidth, "RightPeg_1");
			
			WaypointSequence waypoints_2 = new WaypointSequence(10);
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(25.0, 0.0, 0.0));
			path_2 = PathGenerator.makePath(waypoints_2, config_alt, Constants.kWheelbaseWidth, "RightPeg_2");
		}
		
		@Override
		public void run() {
			robot.setupAutonConfig();
			try {
				Thread.sleep(250); //WAITING FOR SHIFTERS
				RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_0);
				driver.followPathBACKWARDS();
				while(!driver.isDone()) Thread.sleep(20);
				
				robot.rotateDeg(-58.3);
				Thread.sleep(30);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
				driver.followPathBACKWARDS();
				while(!driver.isDone()) Thread.sleep(20);
				
				System.out.println("RELEASE GEAR NOW!");
				//TODO Release Gear
				robot.gearHandler.setDoors(Constants.gearOpen);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
				driver.followPath();
				while(!driver.isDone()) Thread.sleep(20);
				
				robot.rotateDeg(58.3);
				Thread.sleep(30);
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_2);
				driver.followPathBACKWARDS();
			} catch (Exception e) { System.out.println("ERROR: stopping autonomous"); }
		}
		
		

		@Override
		public String getName() {
			// TODO Auto-generated method stub
			return "04-RightPeg";
		}
		
	}
	
	public class CenterPegAlt extends AutonomousMode {
		Path path_0, path_1;
		
		CenterPegAlt(Robot r) {
			super(r);
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 3.0;
			config.max_jerk= 25.0;
			config.max_vel = 4.0;
			
			WaypointSequence waypoint_0 = new WaypointSequence(10);
			waypoint_0.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			waypoint_0.addWaypoint(new WaypointSequence.Waypoint(7.3 , 0.0 , 0.0));
			path_0 = PathGenerator.makePath(waypoint_0, config, Constants.kWheelbaseWidth, "CenterPeg_0");
			
			WaypointSequence waypoint_1 = new WaypointSequence(10);
			waypoint_1.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			waypoint_1.addWaypoint(new WaypointSequence.Waypoint(4.0 , 0.0 , 0.0));
			path_1 = PathGenerator.makePath(waypoint_1, config, Constants.kWheelbaseWidth, "CenterPeg_1");
		}
		
		@Override
		public void run() {
			robot.setupAutonConfig();
			try{ 
				Thread.sleep(250);
				RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_0);
				driver.followPath();
				while (!driver.isDone()) Thread.sleep(20);
				
				robot.gearHandler.setPickup(true);
				Thread.sleep(50);
				
				driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
				driver.followPathBACKWARDS();
				while(!driver.isDone()) Thread.sleep(20);
				
				robot.gearHandler.setPickup(false);
				Thread.sleep(30);
				
				robot.rotateDeg(16.0);
				Thread.sleep(30);
				
				robot.shooter.set(1.0);
				Thread.sleep(320);
				robot.shooter.load(true);
				Thread.sleep(5000);
				robot.shooter.load(false);
				robot.shooter.set(0.0);
				
			} catch (Exception e) {}
		}
		@Override
		public String getName() {
			return "CenterPegAlt";
		}
	}

	public class LeftPegAlt extends AutonomousMode {
		private Path path_0 , path_1 , path_2;
		
		public LeftPegAlt(Robot r)	{
			super(r);
			
			TrajectoryGenerator.Config config =new TrajectoryGenerator.Config();
			config.dt = 0.05;
			config.max_acc = 4.0;
			config.max_jerk= 25.0;
			config.max_vel = 5.0;
			
			TrajectoryGenerator.Config config_alt =new TrajectoryGenerator.Config();
			config_alt.dt = 0.05;
			config_alt.max_acc = 6.0;
			config_alt.max_jerk= 25.0;
			config_alt.max_vel = 9.0;
			
			WaypointSequence waypoints_0 = new WaypointSequence(10);
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(0.0 , 0.0 , 0.0));
			waypoints_0.addWaypoint(new WaypointSequence.Waypoint(6.9 , 0.0 , 0.0));
			path_0 = PathGenerator.makePath(waypoints_0, config, Constants.kWheelbaseWidth, "LeftPeg_0");
			
			WaypointSequence waypoints_1 = new WaypointSequence(10);
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_1.addWaypoint(new WaypointSequence.Waypoint(6.0, 0.0, 0.0));
			path_1 = PathGenerator.makePath(waypoints_1, config, Constants.kWheelbaseWidth, "LeftPeg_1");
			
			WaypointSequence waypoints_2 = new WaypointSequence(10);
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			waypoints_2.addWaypoint(new WaypointSequence.Waypoint(25.0, 0.0, 0.0));
			path_2 = PathGenerator.makePath(waypoints_2, config_alt, Constants.kWheelbaseWidth, "LeftPeg_2");
		}
		
		 @Override
		public void run() {
			 robot.setupAutonConfig();
				try {
					Thread.sleep(250); //WAITING FOR SHIFTERS
					RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_0);
					driver.followPath();
					while(!driver.isDone()) Thread.sleep(20);
					
					robot.rotateDeg(62.5);
					Thread.sleep(30);
					driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
					driver.followPath();
					while(!driver.isDone()) Thread.sleep(20);
					
					System.out.println("RELEASE GEAR NOW!");
					//TODO Release Gear
					robot.gearHandler.setPickup(true);
					driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_1);
					driver.followPath();
					while(!driver.isDone()) Thread.sleep(20);
					
					robot.gearHandler.setPickup(false);
					Thread.sleep(20);
					robot.rotateDeg(-62.5);
					Thread.sleep(30);
					driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans , robot.rightTrans , path_2);
					driver.followPath();
				} catch (Exception e) { System.out.println("ERROR: stopping autonomous"); }
		}
		@Override
		public String getName() {
			return "LegPegAlt";
		}
	}
}