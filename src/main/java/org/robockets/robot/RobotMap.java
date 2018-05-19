package org.robockets.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.team166.chopshoplib.Lidar;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static Victor frontLeft = new Victor(3);
	public static Victor frontRight = new Victor(2);
	public static Victor backLeft = new Victor(1);
	public static Victor backRight = new Victor(0);
	
	public static RobotDrive robotDrive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);

	public static Servo lidarServo = new Servo(4);

	public static Lidar lidar = new Lidar(I2C.Port.kOnboard, 0x62);

	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public static Encoder leftEncoder = new Encoder(0, 1);
	public static Encoder rightEncoder = new Encoder(2, 3);
}
