package org.usfirst.frc.team4201.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro; 
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sensors extends Subsystem{
	
	public ADXRS450_Gyro spartanBoardGyro;
	
	public Sensors(){
		super("Sensors");
		spartanBoardGyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	}
	
	public void updateSmartDashboard(){
		SmartDashboard.putNumber("Spartan Gyro", spartanBoardGyro.getAngle());
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
