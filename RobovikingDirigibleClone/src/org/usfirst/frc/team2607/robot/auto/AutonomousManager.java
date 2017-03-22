package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.Robot;
import org.usfirst.frc.team2607.robot.RobovikingDriveTrainProfileDriver;
import org.usfirst.frc.team2607.robot.RobovikingSRXDriveTrainFollower;

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
		modes.add(new CenterPeg(robot));
		modes.add(new LeftPeg(robot));
		modes.add(new RightPeg(robot));
		modes.add(new StraightTest(robot));
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
	
	public class CenterPeg extends AutonomousMode {
		
		Path path;
		CenterPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
			config.dt = .05;
			config.max_acc = 5.0;//5
			config.max_jerk = 30.0;
			config.max_vel = 7.0;//7
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(7.0, 0.0, 0.0));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "CenterPeg");
		}

		@Override
		public void run() {

			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans, robot.rightTrans, path);
			
			try {
				
				robot.shifter.set(false);
				robot.leftTrans.setHighGear(false , true);
				robot.rightTrans.setHighGear(false , true);
				
				Thread.sleep(50);
			} catch(Exception e) {}
			
			driver.followPathBACKWARDS();
			
			try {
				while(!driver.isDone())
					Thread.sleep(20);
				robot.gearHandler.set(true);
				Thread.sleep(1000);
				robot.leftTrans.set(-100);
				robot.rightTrans.set(100);
				Thread.sleep(499);
				robot.leftTrans.set(0);
				robot.rightTrans.set(0);
				robot.gearHandler.set(false);
			} catch( Exception e) {}
		}

		@Override
		public String getName() {
			
			return "CenterPeg";
		}
		
	}
	

	public class LeftPeg extends AutonomousMode {
		
		Path path;
		
		LeftPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
			config.dt = .05;
			config.max_acc = 5.0;
			config.max_jerk = 30.0;
			config.max_vel = 7.0;
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(6.45, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(8.95 , -1.9 , 5.1));
			//p.addWaypoint(new WaypointSequence.Waypoint(7.5, -1.75, 5.6));  // heading 5.6 turned left instead of right since we follow backwards
			//p.addWaypoint(new WaypointSequence.Waypoint(13.7, 2.3, 1.5));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "LeftPeg");
			
		}

		@Override
		public void run() {
			
			try {
				robot.gearHandler.set(false);
				robot.shifter.set(false);
				robot.leftTrans.setHighGear(false , true);
				robot.rightTrans.setHighGear(false , true);
				Thread.sleep(50);
			} catch(Exception e) {}
			
			System.out.println("running LeftPeg auton....");
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans, robot.rightTrans, path);
			driver.followPathBACKWARDS();
			try {
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
			} catch (Exception e) {
				System.out.println("....LeftPeg path interrupted");
			}
			
			System.out.println("done LeftPeg");
		}

		@Override
		public String getName() {
			return "LeftPeg";
		}
		
	}
	
public class RightPeg extends AutonomousMode {
		
		Path path;
		
		RightPeg(Robot r) {
			super(r);
			TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
			config.dt = .05;
			config.max_acc = 5.0;
			config.max_jerk = 30.0;
			config.max_vel = 7.0;
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(4.5, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(7.1 , 3.6 , 1.55));
			//p.addWaypoint(new WaypointSequence.Waypoint(7.5, -1.75, 5.6));  // heading 5.6 turned left instead of right since we follow backwards
			//p.addWaypoint(new WaypointSequence.Waypoint(13.7, 2.3, 1.5));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "rightPeg");
			
		}

		@Override
		public void run() {
			
			try {
				robot.gearHandler.set(false);
				robot.shifter.set(true);
				robot.leftTrans.setHighGear(false , true);
				robot.rightTrans.setHighGear(false , true);
				Thread.sleep(50);
			} catch(Exception e) {}
			
			System.out.println("running RightPeg auton....");
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans, robot.rightTrans, path);
			driver.followPathBACKWARDS();
			try {
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
			} catch (Exception e) {
				System.out.println("....RightPeg path interrupted");
			}
			
			System.out.println("done RightPeg");
		}

		@Override
		public String getName() {
			return "RightPeg";
		}
		
	}

	public class StraightTest extends AutonomousMode {

		Path path; 
		
		StraightTest(Robot r){
			super(r);
			TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
			config.dt = .05;
			config.max_acc = 7.0;
			config.max_jerk = 30.0;
			config.max_vel = 7.0;
			
			WaypointSequence p = new WaypointSequence(10);
			p.addWaypoint(new WaypointSequence.Waypoint(0.0, 0.0, 0.0));
			p.addWaypoint(new WaypointSequence.Waypoint(6.5,0.0,0.0));
			
			path = PathGenerator.makePath(p, config, Constants.kWheelbaseWidth, "LeftPeg");
		}

		@Override
		public void run() {
			System.out.println("testing a straight line for distance accuracy");
			try {
				robot.shifter.set(true);
				robot.leftTrans.setHighGear(false , true);
				robot.rightTrans.setHighGear(false , true);
				Thread.sleep(50);
			} catch(Exception e) {}

			System.out.println("running StraightTest auton....");
			RobovikingDriveTrainProfileDriver driver = new RobovikingDriveTrainProfileDriver(robot.leftTrans, robot.rightTrans, path);
			driver.followPathBACKWARDS();
			try {
				while (!driver.isDone()) {
					Thread.sleep(20);
				}
			} catch (Exception e) {
				System.out.println("....StraightTest path interrupted");
			}
			
			System.out.println("done StraightTest");

		}

		@Override
		public String getName() {
			return "StraightTest";
		}
		
	}
	
	
	
	
	
	public class DoNothing extends AutonomousMode {
		
		DoNothing(){
			
		}

		@Override
		public void run() {
			System.out.println("Explicitly told not to move");
		}

		@Override
		public String getName() {
			return "DoNothing";
		}
		
	}
	
	public class DoNothingFailsafe extends AutonomousMode {
		
		DoNothingFailsafe(){
			
		}

		@Override
		public void run() {
			System.out.println("This shouldn't be running - Mode 0 selected for some reason");
		}

		@Override
		public String getName() {
			return "DoNothingFailsafe";
		}
		
	}
}