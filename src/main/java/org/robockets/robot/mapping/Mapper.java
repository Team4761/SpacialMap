package org.robockets.robot.mapping;

import org.robockets.commons.geometry.Line;
import org.robockets.commons.geometry.Point;
import org.robockets.commons.math.Vector2D;
import org.robockets.robot.Robot;
import org.robockets.robot.RobotMap;

import java.util.ArrayList;

public class Mapper implements Runnable{

    public Thread thread;
    private ArrayList<Line> map;

    private Point lastPoint;

    private boolean isRunning;

    private int runNumber = 0;

    public Mapper() {
        map = new ArrayList<>();
        isRunning = true;
        thread = new Thread(this);
        thread.start();
    }

    @Override
    public void run() {
        while (true) {
        	if (isRunning) {
        		/*
                Get the robot's position using gyro and encoders. Then, use the lidar to sense how far the wall is away
                */
				Point robotPos = Robot.drivetrain.getDisplacement();

				double direction = getLidarDirection();

				double distance = RobotMap.lidar.getDistance(false);

				Vector2D distanceVector = new Vector2D(distance, direction);

		        Point relativePoint = new Point(distanceVector.getXComponent(), distanceVector.getYComponent());

		        Point realPoint = new Point(robotPos.getX()+relativePoint.getX(), robotPos.getY()+relativePoint.getY());

		        if (lastPoint != null) {
		        	Line line = new Line(lastPoint, realPoint);
		        	map.add(line);
		        }

		        lastPoint = realPoint;
		        runNumber++;

		        if (runNumber % 50 == 0) {
		        	runNumber = 0;
		        	optimize();
		        }
	        }
        }
    }

    public double getLidarDirection() {
    	double robotAngle = RobotMap.gyro.getAngle();
    	double servoAngle = RobotMap.lidarServo.getAngle();

    	return robotAngle - servoAngle;
    }

    public void optimize() {
		// TODO: Implement. This should reduce the number of elements in the list by merging similar lines into larger ones.
    }

    public void stop() { // Unsure if needed
    	isRunning = false;
    }
}
