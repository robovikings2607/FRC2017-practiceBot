package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDPLogger extends Thread {
	
	PowerDistributionPanel pdp;
	String deviceName = "pdp";
	private long curTime , startTime;
	boolean isEnabled;
	PrintWriter logFile = null , logFile2 = null;
	
	public PDPLogger() {
		isEnabled = false;
		pdp = new PowerDistributionPanel();
	}
	
	@Override
	public void run() {
		startTime = System.currentTimeMillis();
		while(true) {
			logEntry();
			try {Thread.sleep(10);} catch(Exception e) {}
		}
	}
	
	private void logEntry() {
		if(isEnabled) {
			curTime = System.currentTimeMillis() - startTime;
			String entry = curTime + " , ";
			entry.concat(pdp.getVoltage() + " , ");
			entry.concat(pdp.getTotalPower() + " , ");
			entry.concat(pdp.getTotalCurrent() + " , ");
			for(int i = 0 ; i < 15 ; i++) {
				entry.concat(pdp.getCurrent(i) + " , ");
			}
			entry.concat(pdp.getCurrent(15) + "");
			logFile.println(entry);
			logFile.flush();
			logFile2.println(entry);
			logFile2.flush();
		}
	}
	
	public void enable() {
		try {
			logFile = new PrintWriter(new File("/home/lvuser/" + deviceName + System.currentTimeMillis() + ".csv"));
			logFile2 = new PrintWriter(new File("/home/lvuser/" + deviceName + ".csv"));
			String header = "Time , Voltage , Total-Power(J) , Total-Current(Amps) , channel-0 , channel-1 , channel-2 , channel-3"
					+ " , channel-4 , channel-5 , channel-6 , channel-7 , channel-8 , channel-9 , channel-10 , channel-11"
					+ " , channel-12 , channel-13 , channel-14 , channel-15";
			logFile.println(header);
			logFile2.println(header);

			isEnabled = true;
		} catch(Exception e) {
			isEnabled = false;
		}
	}
	
	public void disable() {
		if(isEnabled) {
			isEnabled = false;
			logFile.close();
			logFile = null;
			logFile2.close();
			logFile2 = null;
		}
	}
}
