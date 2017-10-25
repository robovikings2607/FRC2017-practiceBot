package org.usfirst.frc.team2607.robot;

import com.ctre.phoenix.MotorControl.SmartMotorController.TalonControlMode;
import com.ctre.phoenix.MotorControl.SmartMotorController.TrajectoryPoint;
import com.ctre.phoenix.MotorControl.CAN.TalonSRX;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.Notifier;

import java.util.concurrent.atomic.AtomicInteger;

import org.usfirst.frc.team2607.robot.Constants;

public class RobovikingSRXDriveTrainFollower extends Thread {

	private TalonSRX leftSRX, rightSRX;				// the SRX's we're driving			
	private AtomicInteger state;			// the state machine control variable
	private TrajectoryPoint[] leftPoints, rightPoints;
	Notifier notifier = new Notifier(new PeriodicRunnable());

	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  leftSRX.processMotionProfileBuffer(); rightSRX.processMotionProfileBuffer(); }
	}
	
	private TalonSRX.MotionProfileStatus leftMPStatus = new TalonSRX.MotionProfileStatus(),
										 rightMPStatus = new TalonSRX.MotionProfileStatus();

	public RobovikingSRXDriveTrainFollower(Transmission leftTrans, Transmission rightTrans, Path p, boolean followBackwards) {
		leftSRX = leftTrans.getMasterSRX();
		rightSRX = rightTrans.getMasterSRX();
		state = new AtomicInteger(0);
		
		Trajectory lt = p.getLeftWheelTrajectory(), rt = p.getRightWheelTrajectory();
		int numPoints = lt.getNumSegments();
		System.out.println("numPoints = " + numPoints);
		leftPoints = new TrajectoryPoint[numPoints+1];
		rightPoints = new TrajectoryPoint[numPoints+1];
		int timeDurMs = (int) (lt.getSegment(0).dt * 1000.0);

		leftPoints[0] = new TrajectoryPoint();
		leftPoints[0].position = 0.0;
		leftPoints[0].zeroPos = true;
		leftPoints[0].velocity = 0.0;
		leftPoints[0].velocityOnly = false;
		leftPoints[0].profileSlotSelect = 0;
		leftPoints[0].timeDurMs = timeDurMs;
		leftPoints[0].isLastPoint = false;
		
		rightPoints[0] = new TrajectoryPoint();
		rightPoints[0] = leftPoints[0];
		
		for (int i=0; i<numPoints; i++) {
			Segment s = lt.getSegment(i);
			double pos, vel;
			if (followBackwards) {
				pos = s.pos;
				vel = s.vel;
			} else {
				pos = -s.pos;
				vel = -s.vel;
			}
			
			leftPoints[i+1] = new TrajectoryPoint();
			leftPoints[i+1].position = Constants.feetToRotations(pos);
			leftPoints[i+1].zeroPos = false;
			leftPoints[i+1].velocity = Constants.feetPerSecondToRPM(vel);
			leftPoints[i+1].velocityOnly = false;
			leftPoints[i+1].profileSlotSelect = 0;
			leftPoints[i+1].timeDurMs = timeDurMs;
			leftPoints[i+1].isLastPoint = false;
			
			s = rt.getSegment(i);
			if (followBackwards) {
				pos = -s.pos;
				vel = -s.vel;
			} else {
				pos = s.pos;
				vel = s.vel;
			}
			
			rightPoints[i+1] = new TrajectoryPoint();
			rightPoints[i+1].position = Constants.feetToRotations(pos);
			rightPoints[i+1].zeroPos = false;
			rightPoints[i+1].velocity = Constants.feetPerSecondToRPM(vel);
			rightPoints[i+1].velocityOnly = false;
			rightPoints[i+1].profileSlotSelect = 0;
			rightPoints[i+1].timeDurMs = timeDurMs;
			rightPoints[i+1].isLastPoint = false;
		}
		
		leftPoints[numPoints].isLastPoint = true;
		rightPoints[numPoints].isLastPoint = true;
		
	}

	public void process() {
		
		int curPoint = 0; 
		
		if (leftSRX.getControlMode() != TalonControlMode.MotionProfile || 
			rightSRX.getControlMode() != TalonControlMode.MotionProfile) return;
		
		leftSRX.getMotionProfileStatus(leftMPStatus);
		rightSRX.getMotionProfileStatus(rightMPStatus);
		
		switch (state.get()) {
			default:					// the default state is just waiting for command to push MP, 
				curPoint = 0; 			// talon MP state could be either Hold or Disable
				break;	
			case 1:						// triggered start
				if (leftMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable && 
				    rightMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable) { 
					leftSRX.clearMotionProfileTrajectories();
					rightSRX.clearMotionProfileTrajectories();
					state.compareAndSet(1,2);		//state += 1;
				}
				leftSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);
				rightSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);
				curPoint = 0;
				break;
			case 2:						// push and start profile
				while (curPoint < leftPoints.length) {
					System.out.println("pushing point " + curPoint);
					leftSRX.pushMotionProfileTrajectory(leftPoints[curPoint]);
					rightSRX.pushMotionProfileTrajectory(rightPoints[curPoint]);
					if (++curPoint == 6) {
						System.out.println("Enabling SRXs");
						leftSRX.set(TalonSRX.SetValueMotionProfile.Enable.value);
						rightSRX.set(TalonSRX.SetValueMotionProfile.Enable.value);
					}
				}
				state.compareAndSet(2, 3);
				System.out.println("going to state 3");
				break;
			case 3:						// MP is running, when we get to end set talon to hold last point
				if (leftMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable && 
					rightMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable) {
					state.compareAndSet(3, 0);
					System.out.println("SRX's disabled, returning to state 0");
				}
				
				if (leftMPStatus.activePointValid && leftMPStatus.activePoint.isLastPoint) {
					leftSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);		
					System.out.println("leftSRX at last point, disabling");
				}
				
				if (rightMPStatus.activePointValid && rightMPStatus.activePoint.isLastPoint) {
					rightSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);
					System.out.println("rightSRX at last point, disabling");
				}
				
			case 10:					// reset state to interrupt and clear the running MP, and go back to wait	
				if (leftMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable && 
					rightMPStatus.outputEnable == TalonSRX.SetValueMotionProfile.Disable) {
					leftSRX.clearMotionProfileTrajectories();
					rightSRX.clearMotionProfileTrajectories();
					state.set(0);
				} else {
					leftSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);
					rightSRX.set(TalonSRX.SetValueMotionProfile.Disable.value);
				}
				break;
		}
	}
	
	@Override
	public void run() {
		System.out.println("Starting RobovikingSRXProfileDriver thread....");
		leftSRX.changeMotionControlFramePeriod(5);
		rightSRX.changeMotionControlFramePeriod(5);
		notifier.startPeriodic(.005);
		try {
			while (true) {
				process();
				try { Thread.sleep(20);} catch (Exception e) {}
			}
		} catch (Exception e) {}
		System.out.println("Leaving RobovikingSRXProfileDriver thread....");
	}
	
	public void runMP() {
		System.out.println("Starting SRX MP");
		state.compareAndSet(0, 1);
	}
	
	public void interruptMP() {
		// if we're running a profile (enabled), interrupt (disable and clear) and go to wait state
		if (state.get() != 0) state.set(10);
	}	
	
	public boolean isMPRunning() {
		return (state.get() != 0);
	}


}
