package org.usfirst.frc.team4201.robot.subsystems;

import org.usfirst.frc.team4201.robot.Robot;
import org.usfirst.frc.team4201.robot.RobotMap;
import org.usfirst.frc.team4201.robot.commands.OI.*;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	public CANTalon[] driveMotors = {
		new CANTalon(RobotMap.driveTrainLeftFront),
		new CANTalon(RobotMap.driveTrainLeftRear),
		new CANTalon(RobotMap.driveTrainRightFront),
		new CANTalon(RobotMap.driveTrainRightRear)
	};
	
	RobotDrive robotDrive = new RobotDrive(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3]);
	
	DoubleSolenoid driveTrainShifters = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.driveTrainShifterForward, RobotMap.driveTrainShifterReverse);
	
	public DriveTrain(){
		super("Drive Train");
		
		// Set Motor Controller Control Mode
		driveMotors[0].changeControlMode(TalonControlMode.PercentVbus);
		driveMotors[1].changeControlMode(TalonControlMode.Follower);
		driveMotors[1].set(driveMotors[0].getDeviceID());
		driveMotors[2].changeControlMode(TalonControlMode.PercentVbus);
		driveMotors[3].changeControlMode(TalonControlMode.Follower);
		driveMotors[3].set(driveMotors[2].getDeviceID());

		// Set Motor Controller Feedback Device
		driveMotors[0].setFeedbackDevice(FeedbackDevice.AnalogEncoder);
		driveMotors[2].setFeedbackDevice(FeedbackDevice.AnalogEncoder);
		
		// Set Motor Controller Peak Output Voltages & Set Motors to Coast
		for(int i = 0; i < 4; i++){
			driveMotors[i].configPeakOutputVoltage(+12.0f, -12.0f);
			driveMotors[i].enableBrakeMode(false);
		}
		
		// Invert Left Motors
		driveMotors[0].setInverted(true);
		driveMotors[1].setInverted(true);
		
		// Set LiveWindow Values		// Need to test/check if this is necessary for CANTalons/Victors
		// LiveWindow.addActuator(this, "Left Front Motor", (SpeedController) driveMotors[0]);
		// LiveWindow.addActuator(this, "Left Rear Motor", (SpeedController) driveMotors[1]);
		// LiveWindow.addActuator(this, "Right Front Motor", (SpeedController) driveMotors[2]);
		// LiveWindow.addActuator(this, "Right Rear Motor", (SpeedController) driveMotors[3]);
	}
	
	public void setDriveModeTankDrive(){
		setDriveTrainOutput(Robot.oi.getLeftJoystickY(), Robot.oi.getRightJoystickY());
	}
	
	public void setDriveModeSplitArcadeDrive(){
		setDriveTrainOutput(Robot.oi.getLeftJoystickY(), Robot.oi.getRightJoystickX());
	}
	
	public void setDriveTrainOutput(double throttle, double angularPower){
		double rightPwm = throttle - angularPower;
        double leftPwm = throttle + angularPower;
        
        // If Math.abs(throttle) + Math.abs(angularPower) > 1, then take the inverse of the excess and apply it to the other side
        if(rightPwm > 1.0){
        	leftPwm -= (rightPwm - 1.0);
        	rightPwm = 1.0;
        } else if(rightPwm < -1.0){
        	leftPwm += (-rightPwm - 1.0);
        	rightPwm = -1.0;
        } else if(leftPwm > 1.0){
        	rightPwm -= (leftPwm - 1.0);
        	leftPwm = 1.0;
        } else if(leftPwm < -1){
        	rightPwm += (-leftPwm - 1.0);
        	leftPwm = -1.0;
        }
        
		robotDrive.tankDrive(leftPwm, rightPwm);
	}

	public void setDriveTrainShiftHigh(){
		driveTrainShifters.set(Value.kForward);
	}
	
	public void setDriveTrainShiftLow(){
		driveTrainShifters.set(Value.kReverse);
	}
	
	public boolean getDriveTrainShiftStatus(){
		// Convert Value to boolean. kOff & kReverse are considered false
		return driveTrainShifters.get() == Value.kForward ? true : false;
	}
	
	public void updateSmartDashboard(){
		SmartDashboard.putBoolean("Drive Train Shift", getDriveTrainShiftStatus());
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new SetDriveModeSplitArcadeDrive());
	}
}
