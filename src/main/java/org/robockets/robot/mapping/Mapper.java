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

    private double previousDisplacement = 0;

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

        		double displacement = previousDisplacement - (RobotMap.leftEncoder.get() + RobotMap.rightEncoder.get()) / 2;

        		/*
                Get the robot's position using gyro and encoders. Then, use the lidar to sense how far the wall is away
                */
				Point robotPos = new Point(displacement * Math.cos(RobotMap.gyro.getAngle()), displacement * Math.sin(RobotMap.gyro.getAngle()));

				previousDisplacement = displacement;

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
	    ArrayList<Line> newMap = new ArrayList<>();

	    for (int i=0; i<map.size(); i++) {
	    	for (int n=i+1; n<map.size(); n++) {
	    		if (almostEqual(map.get(i).getP2().getX(), map.get(n).getP1().getX(), 10)
					    && almostEqual(map.get(i).getP2().getY(), map.get(n).getP1().getY(), 10)) {
	    			// If the two middle points are almost equal

				    // Create average of middle point
				    Point middlePoint = new Point(average(map.get(i).getP2().getX(), map.get(n).getP1().getX()),
						    average(map.get(i).getP2().getY(), map.get(n).getP1().getY()));

				    Point leftPoint = map.get(i).getP1();
				    Point rightPoint = map.get(n).getP2();

				    double leftSide = (middlePoint.getY()-leftPoint.getY())*(rightPoint.getX()-middlePoint.getX());
				    double rightSide = (rightPoint.getY()-middlePoint.getY())*(middlePoint.getX()-leftPoint.getX());

				    if (almostEqual(leftSide, rightSide, 10)) {
				    	newMap.add(new Line(leftPoint, rightPoint));
				    } else {
				    	newMap.add(map.get(i));
				    	newMap.add(map.get(n));
				    }
			    }
		    }

		    map = newMap;
	    }
    }

    public void stop() { // Unsure if needed
    	isRunning = false;
    }

    private boolean almostEqual(double n1, double n2, double percentSensitivity) {
    	if (percentSensitivity <= 0 || percentSensitivity >= 100) {
    		throw new IllegalArgumentException();
	    }

	    double decimalPercent = percentSensitivity / 200.0;
	    double highRange = n2 * (1.0 + decimalPercent);
	    double lowRange = n2 * (1.0 - decimalPercent);
	    return lowRange <= n1 && n1 <= highRange;

    }

    private double average(double n1, double n2) {
    	return (n1+n2)/2;
    }
}
