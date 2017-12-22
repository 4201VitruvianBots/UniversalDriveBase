package org.usfirst.frc.team4201.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// GLOBAL VARIABLES
	public static final int visionCameraWidth = 320;
	public static final int visionCameraHieght = 240;
	public static final int visionCameraCetnerX = 160;
	public static final int visionCameraCenterY = 120;
	public static final double visionCameraAnglesPerPixel = 0.17125;	// With Microsoft Lifecam HD-3000, horizontal FoV is 68.5 degrees, wich gives us 0.17125 angles per pixel with a 320x240 resolution
	
	/*	ADDRESS VALUES
	 *  
	 *  CAN ADDRESSES
	 *  1-19: Major Modules
	 *  +1: PDP
	 *  +2-9: VRMs
	 *  +10-19: PCMs
	 *  
	 *  20-59: Motor Controllers
	 *  +20-24(26): DriveTrain Motors
	 */
	
	// Electrical Modules
	public static final int PDP = 1;
	public static final int VRMOne = 2;
	
	// Pneumatic Modules & their devices
	public static final int PCMOne = 10;
		public static final int driveTrainShifterForward = 0;
		public static final int driveTrainShifterReverse = 1;
	
	// Motor Controllers
	public static final int driveTrainLeftFront = 20;
	public static final int driveTrainLeftRear = 21;
	public static final int driveTrainRightFront = 22;
	public static final int driveTrainRightRear = 23;
	
	// USB ADDRESSES
	public static final int leftJoystick = 0;
	public static final int rightJoystick = 1;
	public static final int xBoxController = 2;
}
