package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

import com.ctre.CANTalon;

public class PIDLogger extends Thread {
	private PrintWriter logFile = null, logFile2 = null;
	private boolean loggingEnabled = false;
	String deviceName;
	private double SP;
	private long curTime, startTime;
	File src;
	
	CANTalon srx;
	
	@Override
	public void run(){
		startTime = System.currentTimeMillis();
		while (true){
			logEntry();
			try {Thread.sleep(10); } catch (Exception e) {}
		}
		
	}
	
	public PIDLogger(CANTalon talonSRX , String dn){
		srx = talonSRX;
		deviceName = dn;
	}
	
	public void updSetpoint(double newSP) {
		SP = newSP;
	}
	
	public void enableLogging(boolean enable) {
	    	boolean okToEnable = enable;
			if (enable && !loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile = null;
	    		}
	    		try {
	    			logFile = new PrintWriter(new File("/home/lvuser/" + deviceName + System.currentTimeMillis() + ".csv"));
	    			logFile2 = new PrintWriter(new File("/home/lvuser/" + deviceName + ".csv"));
	    			String header = "Time,TotalTime,SP,RPM,NativeVel,Err,Rot,NativePos,VIn,VOut,AmpOut,P,I,D,F";
	    			logFile.println(header);
	    			logFile2.println(header);
	    		} catch (Exception e) {
	    			okToEnable = false;
	    		}
	    	} 
	    	
	    	if (!enable && loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile2.close();
	    			logFile = null;
	    			logFile2 = null;
	    		}
	    	}
	    	
	    	loggingEnabled = okToEnable;
	    }
	 
	 public void logEntry() {
	        if (loggingEnabled) {
	        	curTime = System.currentTimeMillis() - startTime;
	        	String line = System.currentTimeMillis() + "," +
	        					curTime + "," +
	        					SP + "," +
	        					srx.getSpeed() + "," + 
	        					srx.getEncVelocity() + "," +
	        				    srx.getClosedLoopError() + "," + 
	        					srx.getPosition() + "," +
	        					srx.getEncPosition() + "," +
	        					srx.getBusVoltage()+ "," + 
	        				    srx.getOutputVoltage() + "," +
	        				    srx.getOutputCurrent() + "," +
	        				    srx.getP() + "," +
	        				    srx.getI() + "," +
	        				    srx.getD() + "," +
	        				    srx.getF();
	        	logFile.println(line);
	        	logFile2.println(line);
	        	logFile.flush();
	        	logFile2.flush();
	        }
	    }

}
