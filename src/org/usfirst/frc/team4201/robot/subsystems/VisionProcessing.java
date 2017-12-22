package org.usfirst.frc.team4201.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class VisionProcessing extends Subsystem implements ITableListener{
	
	public VisionProcessing(){
		super("Vision Processing");
		
		/*  Creates a listener that will notifiy the robot when a particular value is changed
		 *  NetworkTable Flags
		 *  
		 *  This is a 6-bit field used to configure how the listener will act based on how values are update in NetworkTables.
		 *  Values are entered as an integer, but it will be easier to enter the values as a hex using 0x00.
		 * 
		 *  NOTIFY_IMMEDIATE = 0x01;	// ???
		 *  NOTIFY_LOCAL = 0x02;		// Must use when testing locally?
		 *  NOTIFY_NEW = 0x04;			// Notifies when the value is first added
		 *  NOTIFY_DELETE = 0x08;		// Notifies when a value is deleted. Value before deletion is cached and sent as a response.
		 *  NOTIFY_UPDATE = 0x10;		// Notifies when the value is changed
		 *  NOTIFY_FLAGS = 0x20;		// Disables all flag triggers?
		 */
		NetworkTable.getTable("/vision").addTableListenerEx("NumberOfTargets", this, 0x1E);	// Test 0x1E, otherwise use 0x1C
	}
	
	public void updateSmartDashboard(){
		
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void valueChanged(ITable source, String key, Object value,
			boolean isNew) {
		// This function is called whenever the given flag(s) are tripped.
		
	}
}
