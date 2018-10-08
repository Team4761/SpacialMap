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

    private double robotWidth;

    private final double ANGLE_THRESHOLD = 5;

    public Mapper(double robotWidth) {
        map = new ArrayList<>();
        this.robotWidth = robotWidth;
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
	    	// Convert to vector

		    Line line = map.get(i);

		    Vector2D v1 = lineToVector(line);

		    boolean hadMatch = false;
	    	for (int n=i+1; n<map.size(); n++) {
	    		// For each element, test all other elements

			    // Convert to vector

			    Line l2 = map.get(n);

			    Vector2D v2 = lineToVector(l2);

			    double angle = Math.acos(v1.dotProduct(v2)/(v1.getMagnitude()*v2.getMagnitude()));

			    if (angle <= ANGLE_THRESHOLD) {
			    	// Get the inside points
			    	Point p1 = line.getP2();
			    	Point p2 = l2.getP1();

			    	double deltaX = Math.abs(p2.getX() - p1.getX());
			    	double deltaY = Math.abs(p2.getY() - p1.getY());

			    	if (deltaX > robotWidth || deltaY > robotWidth) {
			    		// MERGE
					    hadMatch = true;
					    Line newLine = new Line(line.getP1(), l2.getP2());
					    newMap.add(newLine);
				    }  else {
			    		newMap.add(l2);
				    }
			    } else {
			    	newMap.add(l2);
			    }
		    }

		    if(!hadMatch) {
		    	newMap.add(line);
		    }

		    map = newMap;
	    }
    }

    public void stop() { // Unsure if needed
    	isRunning = false;
    }

    private Vector2D lineToVector(Line line) {
	    Point p1 = line.getP1();
	    Point p2 = line.getP2();

	    double deltaX = p2.getX()-p1.getX();
	    double deltaY = p2.getY()-p1.getY();

		Vector2D vec = new Vector2D(0,0);

		vec.setComponents(deltaX, deltaY);

		return vec;
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
