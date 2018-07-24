package org.robockets.robot.subsystems;

import org.robockets.commons.geometry.Point;
import org.robockets.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A Subsystem for controlling the drivetrain
 * @author Jake Backer and Brian Shin
 */
public class Drivetrain extends Subsystem {

    public static final double MM_TO_CENTER = 200; // The distance from the encoder to the center

    public void initDefaultCommand() {
    	
    }
    
    public void driveArcade(double leftValue, double rightValue) {
    	RobotMap.robotDrive.arcadeDrive(-leftValue, rightValue);
    }
    
    public void stop() {
    	driveArcade(0, 0);
    }


}

