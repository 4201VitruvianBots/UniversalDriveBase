package org.usfirst.frc.team4201.robot.commands;

import org.usfirst.frc.team4201.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Drive straight to a given distance. Distance needs to be calculated, where distance = (encoderCountsPerRev / wheelCircumference) * distanceToGoal. 
 *
 */
public class DriveStraight extends Command {
    static double kP = 0.03;        		// Start with P = 10% of your max output, double until you get a quarter-decay oscillation
    static double kI = 0.0003;				// Start with I = P / 100
    static double kD = 0;       	    	// Start with D = P * 10
    static double kF = 0;       	    	
    static double period = 0.01;
    static double tolerance = 0.1;
	
	Timer settleTimeout;
	boolean lock;
	PIDController leftMotorPID, rightMotorPID;
	PIDOutput leftOutput, rightOutput;
	
    public DriveStraight(double distance) {
        requires(Robot.driveTrain);
        
        // Initialize PIDControllers. Assumes both sides use the same PID values.
    	leftMotorPID = new PIDController(kP, kI, kD, kF, Robot.driveTrain.driveMotors[0], Robot.driveTrain.driveMotors[0], period);
    	leftMotorPID.setContinuous(true);
    	leftMotorPID.setAbsoluteTolerance(tolerance);
    	leftMotorPID.setOutputRange(-1, 1);
    	leftMotorPID.setSetpoint(distance);
    	
    	rightMotorPID = new PIDController(kP, kI, kD, kF, Robot.driveTrain.driveMotors[2], Robot.driveTrain.driveMotors[2], period);
    	rightMotorPID.setContinuous(true);
    	rightMotorPID.setAbsoluteTolerance(tolerance);
    	rightMotorPID.setOutputRange(-1, 1);
    	leftMotorPID.setSetpoint(distance);
    	
    	// Initialize Timer
        settleTimeout = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// Start PID Loops
    	leftMotorPID.enable();
    	rightMotorPID.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Command Execution Time", timeSinceInitialized());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	// When both loops are on target & within tolerance, begin settling timer
    	if(leftMotorPID.onTarget() && rightMotorPID.onTarget() && !lock) { // When you are in range && you are not locked
    		settleTimeout.start();
    		lock = true;
    	} else if((!leftMotorPID.onTarget() || rightMotorPID.onTarget()) && lock){ // When you are outside of range && you are locked
    		settleTimeout.stop();
    		settleTimeout.reset();
    		lock = false;
    	}
    	
    	// If robot has been on target for 1 second && didn't drift out, finish
    	boolean finished = settleTimeout.get() > 1; 
    	
    	// Absolute timeout. Add as a safety measure
    	//if(timeSinceInitialized() > 5);
    	//	finished = true;
    	SmartDashboard.putBoolean("Finished Turn", finished);
    	return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	leftMotorPID.disable();
    	rightMotorPID.disable();
    	Robot.driveTrain.setDriveTrainOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	this.end();
    }
}

