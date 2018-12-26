package usfirst.frc.team6418.robot;

import java.awt.Color;
import java.awt.GraphicsEnvironment;
import java.util.LinkedList;
import java.util.List;

/**
 * This class provides many useful algorithms for robot path planning. It uses optimization techniques and knowledge
 * of robot motion in order to calculate smooth path trajectories, if given only discrete waypoints. The benefit of
 * these optimization algorithms is very efficient path planning that can be used to navigate in real-time.
 * 
 * This class uses a method of gradient descent, and other optimization techniques to produce smooth velocity profiles
 * for both left and right wheels of a differential drive robot.
 * 
 * This class does not attempt to calculate quintic or cubic splines for best fitting a curve. It is for this reason,
 * the algorithm can be ran on embedded devices with very quick computation times.
 * 
 * The output of this function is independent velocity profiles for the left and right wheels of a differential drive
 * chassis. The velocity profiles start and end with 0 velocity and maintain smooth transitions throughout the path.
 * 
 * This algorithm is a port from a similar algorithm running on a robot used for my PhD thesis. I have not fully
 * optimized these functions, so there is room for some improvement.
 * 
 * Initial tests on the 2015 FRC NI RoboRio, the complete algorithm finishes in under 15ms using the Java System Timer
 * for paths with less than 50 nodes.
 *
 * This code has been adapted from https://github.com/KHEngineering/SmoothPathPlanner.git, written by Kevin Harrilal.
 * 
 * @author Kevin Harrilal
 * @author Emily Sturman
 * @version 1.0
 */
public class FalconPathPlanner {
	// path variables
	public double[][] origPath;
	public double[][] nodeOnlyPath;
	public double[][] smoothPath;
	public double[][] leftPath;
	public double[][] rightPath;

	// orig velocity
	public double[][] origCenterVelocity;
	public double[][] origRightVelocity;
	public double[][] origLeftVelocity;

	// smooth velocity
	public double[][] smoothCenterVelocity;
	public double[][] smoothRightVelocity;
	public double[][] smoothLeftVelocity;

	// accumulated heading
	public double[][] heading;

	double totalTime;
	double totalDistance;
	double numFinalPoints;

	double pathAlpha;
	double pathBeta;
	double pathTolerance;

	double velocityAlpha;
	double velocityBeta;
	double velocityTolerance;

	/**
	 * The constructor takes a path of waypoints defined as a double array of column vectors representing the global
	 * Cartesian points of the path in {x,y} coordinates. The waypoints are traveled from one point to the next in
	 * sequence. The units of these coordinates are position units assumed by the user (i.e inch, foot, meters).
     *
	 * @param path waypoint path
	 */
	public FalconPathPlanner(double[][] path) {
		this.origPath = doubleArrayCopy(path);

		// default values; DO NOT MODIFY
		pathAlpha = 0.7;
		pathBeta = 0.3;
		pathTolerance = 0.0000001;

		velocityAlpha = 0.1;
		velocityBeta = 0.3;
		velocityTolerance = 0.0000001;
	}

    /**
     * Prints out double arrays
     *
     * @param path array to print
     */
	public static void print(double[] path) {
		System.out.println("X: \t Y:");

		for (double u: path)
			System.out.println(u);
	}

	/**
	 * Prints Cartesian coordinates to the system output as column vectors
     *
	 * @param path array to print
	 */
	public static void print(double[][] path) {
		System.out.println("X: \t Y:");

		for (double[] u: path)
			System.out.println(u[0]+ "\t" +u[1]);
	}

	/**
	 * Performs a deep copy of a 2D array by looping through each element in the array
	 * 
	 * Big O: O(N x M)
     *
	 * @param arr array to copy
	 * @return copy of arr
	 */
	private static double[][] doubleArrayCopy(double[][] arr) {
		// size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for (int i = 0; i < arr.length; i++) {
			// Resize second dimension of array
			temp[i] = new double[arr[i].length];

			// Copy Contents
            System.arraycopy(arr[i], 0, temp[i], 0, arr[i].length);
		}

		return temp;
	}

	/**
	 * Upsamples the path by linear injection, providing more waypoints along the path.
	 * 
	 * BigO: Order N * injection#
	 * 
	 * @param orig original path
	 * @param numToInject number of points to inject into path
	 * @return path with more waypoints along path
	 */
	private double[][] inject(double[][] orig, int numToInject) {
        double[][] morePoints;

		// create extended 2 Dimensional array to hold additional points
		morePoints = new double[orig.length + ((numToInject) * (orig.length-1))][2];

		int index = 0;

		// loop through original array
		for (int i = 0; i < orig.length-1; i++) {
			// copy first
			morePoints[index][0] = orig[i][0];
			morePoints[index][1] = orig[i][1];
			index++;

			for (int j = 1; j < numToInject+1; j++) {
				// calculate intermediate x points between j and j+1 original points
				morePoints[index][0] = j * ((orig[i+1][0] - orig[i][0]) / (numToInject+1)) + orig[i][0];

				// calculate intermediate y points  between j and j+1 original points
				morePoints[index][1] = j * ((orig[i+1][1] - orig[i][1]) / (numToInject+1)) + orig[i][1];

				index++;
			}
		}

		// copy last
		morePoints[index][0] = orig[orig.length-1][0];
		morePoints[index][1] = orig[orig.length-1][1];

		return morePoints;
	}


	/**
	 * Optimization algorithm which optimizes the data points in a path to create a smooth trajectory.
	 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	 * converge. If this happens, try increasing the tolerance level.
     *
     * weight_data and weight_smooth together determine how smooth the path is
	 * 
	 * Big O: O(N^X), where X is the number of of times the while loop iterates before tolerance is met.
	 * 
	 * @param path waypoint path to smooth
	 * @param weight_data alpha for smoothing path
	 * @param weight_smooth beta for smoothing path
	 * @param tolerance minimum amount of change necessary for the loop to continue
	 * @return smoothed path
	 */
	private double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {
		// copy array
		double[][] newPath = doubleArrayCopy(path);

		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < path.length - 1; i++)
				for (int j=0; j<path[i].length; j++) {
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth
							* (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);
				}
		}

		return newPath;
	}

	/**
	 * Reduces the path into only nodes which change direction. This allows the algorithm to know at what points
	 * the original waypoint vector changes.
	 * 
	 * BigO: O(N + M), where N is length of original path, and M is the number of dimensions in each point in the path
     *
	 * @param path waypoint path
	 * @return path reduced to nodes which change direction
	 */
	private static double[][] nodeOnlyWayPoints(double[][] path) {
		List<double[]> li = new LinkedList<double[]>();

		// save first value
		li.add(path[0]);

		// find intermediate nodes
		for (int i = 1; i < path.length - 1; i++) {
			// calculate direction
			double vector1 = Math.atan2((path[i][1] - path[i-1][1]), path[i][0] - path[i-1][0]);
			double vector2 = Math.atan2((path[i+1][1] - path[i][1]), path[i+1][0] - path[i][0]);

			// determine if both vectors have a change in direction
			if (Math.abs(vector2 - vector1) >= 0.01)
				li.add(path[i]);					
		}

		// save last
		li.add(path[path.length-1]);

		// re-write nodes into new 2D Array
		double[][] temp = new double[li.size()][2];

		for (int i = 0; i < li.size(); i++) {
			temp[i][0] = li.get(i)[0];
			temp[i][1] = li.get(i)[1];
		}	

		return temp;
	}

	/**
	 * Returns velocity as a double array. The first column vector is time, based on the time step, the second vector
	 * is the velocity magnitude.
	 * 
	 * Big O: O(N)
     *
	 * @param smoothPath smoothed path of waypoints
	 * @param timeStep amount of time between each roboRio refresh
	 * @return the velocity as a double array
	 */
	private double[][] velocity(double[][] smoothPath, double timeStep) {
		double[] dxdt = new double[smoothPath.length];
		double[] dydt = new double[smoothPath.length];
		double[][] velocity = new double[smoothPath.length][2];

		// set first instance to zero
		dxdt[0] = 0;
		dydt[0] = 0;
		velocity[0][0] = 0;
		velocity[0][1] = 0;
		heading[0][1] = 0;

		for (int i = 1; i < smoothPath.length; i++) {
			dxdt[i] = (smoothPath[i][0] - smoothPath[i-1][0]) / timeStep;
			dydt[i] = (smoothPath[i][1] - smoothPath[i-1][1]) / timeStep;

			//create time vector
			velocity[i][0] = velocity[i-1][0] + timeStep;
			heading[i][0] = heading[i-1][0] + timeStep;

			//calculate velocity
			velocity[i][1] = Math.sqrt(Math.pow(dxdt[i], 2) + Math.pow(dydt[i], 2));
		}

		return velocity;
	}

	/**
	 * Optimize velocity by minimizing the error distance at the end of travel.
	 * When this function converges, the fixed velocity vector will be smooth, start
	 * and end with 0 velocity, and travel the same final distance as the original
	 * un-smoothed velocity profile. This algorithm may never converge. If this happens, reduce tolerance.
	 * 
	 * @param smoothVelocity two-dimensional array of the smooth path velocity (obtained via velocity() method)
	 * @param origVelocity starting velocity of robot
	 * @param tolerance minimum change in error between origVelocity and smoothVelocity
	 * @return fixed velocity
	 */
	private double[][] velocityFix(double[][] smoothVelocity, double[][] origVelocity, double tolerance) {
		/* Pseudo
		 * 1. Find error between original velocity and smooth velocity
		 * 2. Keep increasing the velocity between the first and last node of the smooth velocity by a small amount
		 * 3. Recalculate the difference, stop if threshold is met or repeat step 2 until the final threshold is met.
		 * 3. Return the updated smoothVelocity
		 */

		// calculate error difference
		double[] difference = errorSum(origVelocity,smoothVelocity);


		// copy smooth velocity into new Vector
		double[][] fixVel = new double[smoothVelocity.length][2];

		for (int i = 0; i < smoothVelocity.length; i++) {
			fixVel[i][0] = smoothVelocity[i][0];
			fixVel[i][1] = smoothVelocity[i][1];
		}

		double increase = 0.0;
		while (Math.abs(difference[difference.length-1]) > tolerance) {
		    // TODO:@emily Figure out why they divide by 1 first
			increase = difference[difference.length-1] / 1 / 50;

			for (int i = 1; i < fixVel.length - 1; i++)
				fixVel[i][1] = fixVel[i][1] - increase;

			difference = errorSum(origVelocity, fixVel);
		}

		// fixVel =  smoother(fixVel, 0.001, 0.001, 0.0000001);
		return fixVel;
	}

	/**
	 * Calculates the integral of the smooth velocity term and compares it to the integral of the
	 * original velocity term. In essence we are comparing the total distance by the original velocity path and 
	 * the smooth velocity path to ensure that as we modify the smooth velocity it still covers the same distance
	 * as was intended by the original velocity path.
	 * 
	 * Big O: O(N)
     *
	 * @param origVelocity original velocity term (two-dimensional array of doubles)
	 * @param smoothVelocity smooth velocity term (two-dimensional array of doubles)
	 * @return difference between origVelocity and smoothVelocity
	 */
	private double[] errorSum(double[][] origVelocity, double[][] smoothVelocity) {
		// copy vectors
		double[] tempOrigDist = new double[origVelocity.length];
		double[] tempSmoothDist = new double[smoothVelocity.length];
		double[] difference = new double[smoothVelocity.length];


		double timeStep = origVelocity[1][0] - origVelocity[0][0];

		// copy first elements
		tempOrigDist[0] = origVelocity[0][1];
		tempSmoothDist[0] = smoothVelocity[0][1];


		// calculate difference
		for (int i = 1; i < origVelocity.length; i++) {
			tempOrigDist[i] = origVelocity[i][1] * timeStep + tempOrigDist[i-1];
			tempSmoothDist[i] = smoothVelocity[i][1] * timeStep + tempSmoothDist[i-1];

			difference[i] = tempSmoothDist[i] - tempOrigDist[i];
		}

		return difference;
	}

	/**
	 * Calculates the optimal parameters for determining what amount of nodes to inject into the path
	 * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
	 * results for the final smooth path.
	 * 
	 * Big O: O(1)
	 * 	
	 * @param numNodeOnlyPoints number of nodes in which the robot changes direction
	 * @param maxTimeToComplete time in which robot must complete path
	 * @param timeStep amount of time between roboRio updates
     * @return int array containing parameters
	 */
	private int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep) {
		int first = 0;
		int second = 0;
		int third = 0;

		double oldPointsTotal = 0;
		numFinalPoints  = 0;
		int[] ret = null;
		double totalPoints = maxTimeToComplete/timeStep;

		if (totalPoints < 100) {
			double pointsFirst = 0;
			double pointsTotal = 0;

			for (int i = 4; i <= 6; i++)
				for (int j = 1; j <= 8; j++) {
					pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
					pointsTotal = j * (pointsFirst - 1) + pointsFirst;

					if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {
						first = i;
						second = j;
						numFinalPoints = pointsTotal;
						oldPointsTotal = pointsTotal;
					}
				}

			ret = new int[] {first, second, third};
		} else {
			double pointsFirst = 0;
			double pointsSecond = 0;
			double pointsTotal = 0;

			for (int i = 1; i <= 5; i++)
				for (int j = 1; j <= 8; j++)
					for (int k = 1; k < 8; k++) {
						pointsFirst = i * (numNodeOnlyPoints-1) + numNodeOnlyPoints;
						pointsSecond = j * (pointsFirst - 1) + pointsFirst;
						pointsTotal = k * (pointsSecond - 1) + pointsSecond;

						if (pointsTotal <= totalPoints) {
							first = i;
							second = j;
							third = k;
							numFinalPoints = pointsTotal;
						}
					}

			ret = new int[] {first, second, third};
		}

		return ret;
	}

    /**
     * Calculates the left and right wheel paths based on robot track width
     *
     * Big O: O(N)
     *
     * @param smoothPath center smooth path of robot
     * @param robotTrackWidth width between left and right wheels of robot of skid steer chassis.
     */
	private void leftRight(double[][] smoothPath, double robotTrackWidth) {
		double[][] leftPath = new double[smoothPath.length][2];
		double[][] rightPath = new double[smoothPath.length][2];
		double[][] gradient = new double[smoothPath.length][2];

		for (int i = 0; i < smoothPath.length - 1; i++)
			gradient[i][1] = Math.atan2(smoothPath[i+1][1] - smoothPath[i][1], smoothPath[i+1][0] - smoothPath[i][0]);

		gradient[gradient.length-1][1] = gradient[gradient.length-2][1];

		for (int i = 0; i < gradient.length; i++) {
			leftPath[i][0] = (robotTrackWidth/2 * Math.cos(gradient[i][1] + Math.PI/2)) + smoothPath[i][0];
			leftPath[i][1] = (robotTrackWidth/2 * Math.sin(gradient[i][1] + Math.PI/2)) + smoothPath[i][1];

			rightPath[i][0] = robotTrackWidth/2 * Math.cos(gradient[i][1] - Math.PI/2) + smoothPath[i][0];
			rightPath[i][1] = robotTrackWidth/2 * Math.sin(gradient[i][1] - Math.PI/2) + smoothPath[i][1];

			// convert to degrees 0 to 360 where 0 degrees is + X - axis, accumulated to align with WPI sensor
			double deg = Math.toDegrees(gradient[i][1]);
			gradient[i][1] = deg;

			if (i > 0) {
				if ((deg - gradient[i-1][1]) > 180)
					gradient[i][1] = -360 + deg;

				if ((deg - gradient[i-1][1]) < -180)
					gradient[i][1] = 360 + deg;
			}
		}

		this.heading = gradient;
		this.leftPath = leftPath;
		this.rightPath = rightPath;
	}
	
	/**
	 * Returns the first column of a 2D array of doubles
	 *
	 * @param arr 2D array of doubles
	 * @return array of doubles representing the 1st column of the initial parameter
	 */
	public static double[] getXVector(double[][] arr) {
		double[] temp = new double[arr.length];

		for (int i = 0; i < temp.length; i++)
			temp[i] = arr[i][0];

		return temp;
	}

	/**
	 * Returns the second column of a 2D array of doubles
	 * 
	 * @param arr 2D array of doubles
	 * @return array of doubles representing the 1st column of the initial parameter
	 */
	public static double[] getYVector(double[][] arr) {
		double[] temp = new double[arr.length];

		for (int i = 0; i < temp.length; i++)
			temp[i] = arr[i][1];

		return temp;
	}

    /**
     * Transposes array (rows to columns, columns to rows)
	 *
     * @param arr 2D array of doubles to be transposed
     * @return transposed array of double arrays
     */
	public static double[][] transposeVector(double[][] arr) {
		double[][] temp = new double[arr[0].length][arr.length];

		for (int i = 0; i < temp.length; i++)
			for (int j = 0; j < temp[i].length; j++)
				temp[i][j] = arr[j][i];

		return temp;		
	}

	/**
	 * Sets pathAlpha to specified double
	 *
	 * @param alpha double to set pathAlpha to
	 */
	public void setPathAlpha(double alpha) {
		pathAlpha = alpha;
	}

	/**
	 * Sets pathBeta to specified double
	 *
	 * @param beta double to set pathBeta to
	 */
	public void setPathBeta(double beta) {
		pathBeta = beta;
	}

	/**
	 * Sets pathTolerance to specified double
	 *
	 * @param tolerance double to set pathTolerance to
	 */
	public void setPathTolerance(double tolerance) {
		pathTolerance = tolerance;
	}

	/**
	 * Calculates a smooth path based on the program parameters. If the user doesn't set any parameters,
     * it uses the defaults optimized for most cases. The results will be saved into the corresponding class
     * members. The user can then access .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity,
     * .smoothRightVelocity, .smoothLeftVelocity as needed.
	 * 
	 * After calling this method, the user only needs to pass .smoothRightVelocity[1], .smoothLeftVelocity[1] to the
     * corresponding speed controllers on the Robot, and step through each setPoint.
	 * 
	 * @param totalTime time the user wishes to complete the path in seconds. This is the maximum amount of time the
	 *                  robot is allowed to take to traverse the path.
	 * @param timeStep frequency at which the robot controller is running on the robot
	 * @param robotTrackWidth distance between left and right side wheels of a skid steer chassis
	 */
	public void calculate(double totalTime, double timeStep, double robotTrackWidth) {
	    /* Pseudo
		 * 1. Reduce input waypoints to only essential (direction changing) node points
		 * 2. Calculate how many total data points we need to satisfy the controller for "playback"
		 * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number 
		 *    of data points, and which follows the waypoint path.
		 * 4. Calculate left and right wheel paths by calculating parallel points at each data point
		 */

		// first find only direction changing nodes
		nodeOnlyPath = nodeOnlyWayPoints(origPath);

		// figure out how many nodes to inject
		int[] inject = injectionCounter2Steps(nodeOnlyPath.length, totalTime, timeStep);

		// iteratively inject and smooth the path
		for (int i = 0; i < inject.length; i++) {
			if (i == 0) {
				smoothPath = inject(nodeOnlyPath,inject[0]);
				smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);	
			} else {
				smoothPath = inject(smoothPath,inject[i]);
				smoothPath = smoother(smoothPath, 0.1, 0.3, 0.0000001);	
			}
		}

		// calculate left and right path based on center path
		leftRight(smoothPath, robotTrackWidth);

		origCenterVelocity = velocity(smoothPath, timeStep);
		origLeftVelocity = velocity(leftPath, timeStep);
		origRightVelocity = velocity(rightPath, timeStep);

		// copy smooth velocities into fix velocities
		smoothCenterVelocity = doubleArrayCopy(origCenterVelocity);
		smoothLeftVelocity = doubleArrayCopy(origLeftVelocity);
		smoothRightVelocity = doubleArrayCopy(origRightVelocity);

		// set final vel to zero
		smoothCenterVelocity[smoothCenterVelocity.length-1][1] = 0.0;
		smoothLeftVelocity[smoothLeftVelocity.length-1][1] = 0.0;
		smoothRightVelocity[smoothRightVelocity.length-1][1] = 0.0;

		// smooth velocity with zero final V
		smoothCenterVelocity = smoother(smoothCenterVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothLeftVelocity = smoother(smoothLeftVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothRightVelocity = smoother(smoothRightVelocity,velocityAlpha, velocityBeta, velocityTolerance);

		// fix velocity distance error
		smoothCenterVelocity = velocityFix(smoothCenterVelocity, origCenterVelocity, 0.0000001);
		smoothLeftVelocity = velocityFix(smoothLeftVelocity, origLeftVelocity, 0.0000001);
		smoothRightVelocity = velocityFix(smoothRightVelocity, origRightVelocity, 0.0000001);
	}

    /**
     * Graphs path of waypoints, smoothed path, and left and right wheel trajectories on field
     */
    public void graphPath() {
        // Creates a blank image
        double[][] pathPoints = this.nodeOnlyPath;
        FalconLinePlot fig = new FalconLinePlot(new double[][] {{0.0, 0.0}});

        createField(fig);

        // Add waypoint path to graph
        fig.addData(this.nodeOnlyPath, Color.blue, Color.green);

        // Add all other paths to graph
        fig.addData(this.smoothPath, Color.red, Color.blue);
        fig.addData(this.leftPath, Color.magenta);
        fig.addData(this.rightPath, Color.magenta);

        // Add robot box around 1st and last points
        double beginX = pathPoints[0][0];
        double beginY = pathPoints[0][1];
        double endX = pathPoints[pathPoints.length-1][0];
        double endY = pathPoints[pathPoints.length-1][1];
        double[] penultPt = pathPoints[pathPoints.length-2];
        double[][] endPos;

        double[][] beginPos = new double[][] {
                {beginX + 33.0/24, beginY + 7.0/6},
                {beginX + 33.0/24, beginY - 7.0/6},
                {beginX - 33.0/24, beginY - 7.0/6},
                {beginX - 33.0/24, beginY + 7.0/6},
                {beginX + 33.0/24, beginY + 7.0/6},
        };

        if (endX - penultPt[0] == 0) {
            endPos = new double [][] {
                    {endX + 7.0/6, endY + 33.0/24},
                    {endX + 7.0/6, endY - 33.0/24},
                    {endX - 7.0/6, endY - 33.0/24},
                    {endX - 7.0/6, endY + 33.0/24},
                    {endX + 7.0/6, endY + 33.0/24},
            };
        } else {
            // Set slope to rise / run, or (y1 - y2) / (x1 - x2)
            double m = (endY - penultPt[1]) / (endX - penultPt[0]);

            if (m == 0) {
                endPos = new double[][] {
                        {endX + 16.5 / 12, endY + 14.0 / 12},
                        {endX + 16.5 / 12, endY - 14.0 / 12},
                        {endX - 16.5 / 12, endY - 14.0 / 12},
                        {endX - 16.5 / 12, endY + 14.0 / 12},
                        {endX + 16.5 / 12, endY + 14.0 / 12},
                };
            } else {
                double[] endPoint = new double[] {endX, endY};
                double[] point1 = findXYCoordinates(-1 / m, endPoint, 14.0 / 12);
                double[] point2 = findXYCoordinates(m, point1, 16.5 / 12);
                double[] point3 = findXYCoordinates(-1 / m, point2, -28.0 / 12);
                double[] point4 = findXYCoordinates(m, point3, -33.0 / 12);
                double[] point5 = findXYCoordinates(-1 / m, point4, 28.0 / 12);
                double[] point6 = findXYCoordinates(m, point5, 16.5 / 12);

                endPos = new double[][] {
                        point1,
                        point2,
                        point3,
                        point4,
                        point5,
                        point6
                };
            }
        }

        fig.addData(beginPos, Color.blue);
        fig.addData(endPos, Color.blue);
        fig.repaint();
    }

    /**
     * Calculates X and Y coordinates when given a starting point, distance, and slope of line between the two points
     *
     * @param m is the slope of line created by starting and ending point
     * @param startPoint is the {X, Y} coordinates of starting point
     * @param lineLength is the distance between starting point and ending point
     * @return {X, Y} coordinates of ending point
     */
    private double[] findXYCoordinates(double m, double[] startPoint, double lineLength) {

        double angle = Math.atan(Math.abs(m));
        double x = Math.cos(angle) * lineLength;
        double y = x * m;

        return new double[] {startPoint[0] + x, startPoint[1] + y};
    }

    /**
     * Adds markers to field graph
     *
     * @param fig is the graph to be edited
     */
    private void createField(FalconLinePlot fig) {
        fig.yGridOn();
        fig.xGridOn();
        fig.setYLabel("Y (feet)");
        fig.setXLabel("X (feet)");
        fig.setTitle("Top Down View of FRC Field (37ft x 27ft) \n shows global"
                + "position of robot path, along with left and right wheel trajectories");


        // Forces graph to show field dimensions of 27ft x 54ft
        double fieldWidth = 27.0;
        double fieldLength = 54.0;
        fig.setXTic(0, fieldLength, 1);
        fig.setYTic(0, fieldWidth, 1);

        // Auto lines (10 feet from each side)
        double[][] autoLineLeft = new double[][] {{10,0}, {10, fieldWidth}};
        double[][] autoLineRight = new double[][] {{fieldLength - 10,0}, {fieldLength - 10, fieldWidth}};
        fig.addData(autoLineLeft, Color.black);
        fig.addData(autoLineRight, Color.black);

        // Exchange zones
        double[][] leftExchangeZone = new double[][]{
                {0, fieldWidth - 8 - 5.69/12},
                {3, fieldWidth - 8 - 5.69/12},
                {3, fieldWidth - 12 - 5.69/12},
                {0, fieldWidth - 12 - 5.69/12},
                {0, fieldWidth - 8 - 5.69/12},
        };

        double[][] rightExchangeZone = new double[][]{
                {fieldLength, 8 + 5.69/12},
                {fieldLength - 3, 8 + 5.69/12},
                {fieldLength - 3, 12 + 5.69/12},
                {fieldLength, 12 + 5.69/12},
                {fieldLength, 8 + 5.69/12},
        };

        fig.addData(leftExchangeZone, Color.red);
        fig.addData(rightExchangeZone, Color.blue);

        // Power cube zones
        double[][] leftPowerCubeZone = new double[][]{
                {12 - 1.0/3, fieldWidth - 11.625},
                {12 - 1.0/3, fieldWidth - 15.375},
                {8.5 - 1.0/3, fieldWidth - 15.375},
                {8.5 - 1.0/3, fieldWidth - 11.625},
                {12 - 1.0/3, fieldWidth - 11.625},
        };

        double[][] rightPowerCubeZone = new double[][]{
                {fieldLength - 12 + 1.0/3, fieldWidth - 11.625},
                {fieldLength - 12 + 1.0/3, fieldWidth - 15.375},
                {fieldLength - 8.5 + 1.0/3, fieldWidth - 15.375},
                {fieldLength - 8.5 + 1.0/3, fieldWidth - 11.625},
                {fieldLength - 12 + 1.0/3, fieldWidth - 11.625},
        };

        fig.addData(leftPowerCubeZone, Color.red);
        fig.addData(rightPowerCubeZone, Color.blue);

        // Switches and borders
        double[][] leftSwitch = new double[][]{
                {12, fieldWidth - 7.5},
                {16, fieldWidth - 7.5},
                {16, fieldWidth - 19.5},
                {12, fieldWidth - 19.5},
                {12, fieldWidth - 7.5},
                {16, fieldWidth - 7.5},
                {16, fieldWidth - 10.5},
                {12, fieldWidth - 10.5},
                {12, fieldWidth - 16.5},
                {16, fieldWidth - 16.5},
        };

        double[][] leftSwitchFence = new double[][]{
                {12 - 1.0/3, fieldWidth - (7.5 - 9.5/24)},
                {16 + 1.0/3, fieldWidth - (7.5 - 9.5/24)},
                {16 + 1.0/3, fieldWidth - (19.5 + 9.5/24)},
                {12 - 1.0/3, fieldWidth - (19.5 + 9.5/24)},
                {12 - 1.0/3, fieldWidth - (7.5 - 9.5/24)},
        };

        double[][] rightSwitch = new double[][]{
                {fieldLength - 12, fieldWidth - 7.5},
                {fieldLength - 16, fieldWidth - 7.5},
                {fieldLength - 16, fieldWidth - 19.5},
                {fieldLength - 12, fieldWidth - 19.5},
                {fieldLength - 12, fieldWidth - 7.5},
                {fieldLength - 16, fieldWidth - 7.5},
                {fieldLength - 16, fieldWidth - 10.5},
                {fieldLength - 12, fieldWidth - 10.5},
                {fieldLength - 12, fieldWidth - 16.5},
                {fieldLength - 16, fieldWidth - 16.5},
        };

        double[][] rightSwitchFence = new double[][]{
                {fieldLength - (12 - 1.0/3), fieldWidth - 7.5 + 9.5/24},
                {fieldLength - (16 + 1.0/3), fieldWidth - 7.5 + 9.5/24},
                {fieldLength - (16 + 1.0/3), fieldWidth - 19.5 - 9.5/24},
                {fieldLength - (12 - 1.0/3), fieldWidth - 19.5 - 9.5/24},
                {fieldLength - (12 - 1.0/3), fieldWidth - 7.5 + 9.5/24},
        };

        fig.addData(leftSwitch, Color.black);
        fig.addData(leftSwitchFence, Color.black);
        fig.addData(rightSwitch, Color.black);
        fig.addData(rightSwitchFence, Color.black);

        // Field border
        double[][] border = new double[][]{
                {3, 0},
                {0, 2 + 5.69/12},
                {0, fieldWidth - (2 + 5.69/12)},
                {3, fieldWidth},
                {fieldLength - 3, fieldWidth},
                {fieldLength, fieldWidth - (2 + 5.69/12)},
                {fieldLength, 2 + 5.69/12},
                {fieldLength - 3, 0},
                {3, 0},
        };

        fig.addData(border, Color.black);

        // Platform
        double[][] platform = new double[][]{
                {fieldLength/2 + 5.0 + 1.0/4, fieldWidth/2 + (5 + 5.0/12)},
                {fieldLength/2 + 5.0 + 1.0/4, fieldWidth/2 - (5 + 5.0/12)},
                {fieldLength/2 - 5.0 - 1.0/4, fieldWidth/2 - (5 + 5.0/12)},
                {fieldLength/2 - 5.0 - 1.0/4, fieldWidth/2 + (5 + 5.0/12)},
                {fieldLength/2 + 5.0 + 1.0/4, fieldWidth/2 + (5 + 5.0/12)},
        };

        fig.addData(platform, Color.black);

        // Scale
        double[][] scale = new double[][]{
                {fieldLength/2 - 2, fieldWidth/2 - 7.5},
                {fieldLength/2 + 2, fieldWidth/2 - 7.5},
                {fieldLength/2 + 2, fieldWidth/2 - 4.5},
                {fieldLength/2 - 2, fieldWidth/2 - 4.5},
                {fieldLength/2 - 2, fieldWidth/2 - 7.5},
                {fieldLength/2 - 2, fieldWidth/2 - 4.5},
                {fieldLength/2 - 1.0, fieldWidth/2 - 4.5},
                {fieldLength/2 - 1.0, fieldWidth/2 + 4.5},
                {fieldLength/2 + 1.0, fieldWidth/2 + 4.5},
                {fieldLength/2 + 1.0, fieldWidth/2 - 4.5},
                {fieldLength/2 + 1.0, fieldWidth/2 + 4.5},
                {fieldLength/2, fieldWidth/2 + 4.5},
                {fieldLength/2 - 2, fieldWidth/2 + 4.5},
                {fieldLength/2 - 2, fieldWidth/2 + 7.5},
                {fieldLength/2 - 2, fieldWidth/2 + 4.5},
                {fieldLength/2 + 2, fieldWidth/2 + 4.5},
                {fieldLength/2 + 2, fieldWidth/2 + 7.5},
                {fieldLength/2 - 2, fieldWidth/2 + 7.5},
        };

        fig.addData(scale, Color.black);

        double[][] powerCube;
        double powerCubeWidth = fieldWidth/2 + 6 + 4.75/12 * 28.1/12;
        double powerCubeHeight = fieldWidth/2 + 5 + 3.75/12 * 28.1/12;

        for (int i = 0; i < 6; i++) {
            powerCube = new double[][]{
                    {16 + 1.0/3, powerCubeWidth - i},
                    {16 + 1.0/3, powerCubeHeight - i},
                    {17 + 5.0/12, powerCubeHeight - i},
                    {17 + 5.0/12, powerCubeWidth - i},
                    {16 + 1.0/3, powerCubeWidth - i},
            };

            fig.addData(powerCube, Color.yellow);
        }
    }

	// main program
	public static void main(String[] args) {
        // Enable this to true to emulate roboRio environment
		// System.setProperty("java.awt.headless", "true");

        double[][] middleToLeft = new double[][] {
                {33.0/24, 13},
                {5, 13},
                {7, 18},
                {10,18},
        };
        double[][] middleToRight = new double[][] {
                {33.0/24, 13},
                {5, 13},
                {7, 9},
                {10,9},
        };
        double[][] leftToSwitch = new double[][] {
                {33.0/24, 27.0 - (2 + 5.69/12) - 14.0/12},
                {14, 27.0 - (2 + 5.69/12) - 14.0/12},
                {14, 20 + 16.5/12},
        };
        double[][] rightToSwitch = new double[][] {
                {33.0/24, 2 + 5.69/12 + 14.0/12},
                {14, 2 + 5.69/12 + 14.0/12},
                {14, 7 - 16.5/12},
        };
        double[][] leftToLeftScale = new double[][] {
                {33.0/24, 27.0 - (2 + 5.69/12) - 14.0/12},
                {10, 27.0 - (2 + 5.69/12) - 14.0/12},
                {27, 25},
                {27, 22.5},
        };
        double[][] leftToRightScale = new double[][] {
                {33.0/24, 27.0 - (2 + 5.69/12) - 14.0/12},
                {22, 27.0 - (2 + 5.69/12) - 14.0/12},
                {19, 2},
                {27, 1},
                {27, 4},
        };
        double[][] rightToRightScale = new double[][] {
                {33.0/24, 2 + 5.69/12 + 14.0/12},
                {10, 2 + 5.69/12 + 14.0/12},
                {27, 2},
                {27, 4.5},
        };
        double[][] rightToLeftScale = new double[][] {
                {33.0/24, 2 + 5.69/12 + 14.0/12},
                {20, 2 + 5.69/12 + 14.0/12},
                {19, 21},
                {23.5, 21},
        };

		double totalTime = 8; // seconds
		double timeStep = 0.1; // period of control loop on roboRio, seconds
		double robotTrackWidth = 2; // distance between left and right wheels, feet

		final FalconPathPlanner path = new FalconPathPlanner(rightToLeftScale);
		path.calculate(totalTime, timeStep, robotTrackWidth);

		if (!GraphicsEnvironment.isHeadless()) {
            // Velocity
            FalconLinePlot velocityFig = new FalconLinePlot(path.smoothCenterVelocity, Color.blue, Color.blue);
            velocityFig.yGridOn();
            velocityFig.xGridOn();
            velocityFig.setYLabel("Velocity (ft/sec)");
            velocityFig.setXLabel("Time (seconds)");
            velocityFig.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
            velocityFig.addData(path.smoothRightVelocity, Color.magenta);
            velocityFig.addData(path.smoothLeftVelocity, Color.cyan);
            velocityFig.repaint();

			// graph chosen path
			path.graphPath();
		}
	}
}




