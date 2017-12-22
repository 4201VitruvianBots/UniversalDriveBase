package org.usfirst.frc.team4201.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4201.robot.Robot;

/** Turn the robot to a given angle.
 * 	
 */
public class DriveTurnInPlace extends PIDCommand{
    static double kP = 0.03;        		// Start with P = 10% of your max output, double until you get a quarter-decay oscillation
    static double kI = 0.0003;				// Start with I = P / 100
    static double kD = 0;       	    	// Start with D = P * 10
    static double kF = 0;
    static double period = 0.01;
    static double tolerance = 0.1;
    
    double setpoint, getAngle;
    Timer settleTimeout;
    boolean lock = false;
    
    public DriveTurnInPlace(double angle){
        super("DriveTurnInPlace", kP, kI, kD, period);
        requires(Robot.driveTrain);
        requires(Robot.sensors);
        
        getPIDController().setContinuous(true);
        getPIDController().setAbsoluteTolerance(tolerance);
        getPIDController().setOutputRange(-1, 1);
        //getPIDController().setPID(kP, kI, kD, kF);	// Use this if you want to include the feed-forward term

        this.setpoint = angle;
        
        settleTimeout = new Timer();
    }
    
    @Override
    protected double returnPIDInput() {
        // Use the Spartanboard Gyro as your reference
    	getAngle = Robot.sensors.spartanBoardGyro.getAngle();
        return getAngle;
    }
    
    @Override
    protected void usePIDOutput(double output) {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("PID Output", output); // is this an angle value, or a percentage? Output may need to be negative
        Robot.driveTrain.setDriveTrainOutput(0, output);
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
        // Use this to test/tune PID loop through SmartDashboard
    	if(!SmartDashboard.containsKey("kP"))
    		SmartDashboard.putNumber("kP", kP);
    	if(!SmartDashboard.containsKey("kI"))
    		SmartDashboard.putNumber("kI", kI);
    	if(!SmartDashboard.containsKey("kD"))
    		SmartDashboard.putNumber("kD", kD);
        //if(!SmartDashboard.containsKey("kF"))
        	//SmartDashboard.putNumber("kF", kF);
    	
        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);
        //kF = SmartDashboard.getNumber("kF", kF);
        
        getPIDController().enable();
        getPIDController().setSetpoint(setpoint);
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Command Execution Time", timeSinceInitialized());
    }
    
    @Override
    protected boolean isFinished() {
    	// Place these values onto the SmartDashboard for testing/tuning
    	SmartDashboard.putNumber("Settle Timer", settleTimeout.get());
    	SmartDashboard.putBoolean("Lock Value: ", lock);
    	
    	// Robot will stop adjusting angle when its angle is within tolerance range for 1 second
    	if(Math.abs(setpoint - getAngle) < tolerance && !lock) { // When you are in range && you are not locked
    		settleTimeout.start();
    		lock = true;
    	} else if(Math.abs(setpoint - getAngle) >= 0.1 && lock){ // When you are outside of range && you are locked
    		settleTimeout.stop();
    		settleTimeout.reset();
    		lock = false;
    	}
    	
    	boolean finished = settleTimeout.get() > 1; 
    	
    	// Absolute timeout. Add as a safety measure
    	//if(timeSinceInitialized() > 5);
    	//	finished = true;
    	SmartDashboard.putBoolean("Finished Turn", finished);
    	return finished;
    }
    
    // Called once after isFinished returns true
    protected void end() {
    	getPIDController().disable();
        Robot.driveTrain.setDriveTrainOutput(0, 0);
    	settleTimeout.stop();
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
}