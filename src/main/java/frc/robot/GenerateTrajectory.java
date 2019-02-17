/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Generates a spline/trajectory for the robot to follow
 */
public class GenerateTrajectory {

	public static boolean forceRegen = false;

	private static final double TIME_STEP = RobotSettings.TIME_STEP; // seconds
	private static final double MAX_VELOCITY = RobotSettings.MAX_VELOCITY;
	private static final double MAX_ACCELERATION = RobotSettings.MAX_ACCELERATION;
	private static final double MAX_JERK = RobotSettings.MAX_JERK;
	private static final double WHEELBASE_WIDTH = RobotSettings.WHEELBASE_WIDTH;

	private static final String SAVE_DIR = RobotSettings.SAVE_DIR_TRAJECTORIES;

	/**
	 * Generates a trajectory with the supplied waypoints and saves it with a given
	 * name.
	 * 
	 * @param name   - The name to save the trajectory as.
	 * @param points - The waypoints the trajectory must satisfy
	 * @return The thread that is generating the trajectory.
	 */
	public static Thread execute(String name, Waypoint... points) {
		if (points.length == 0) {
			return null;
		}

		if (!new File(SAVE_DIR + name).exists()) {
			new File(SAVE_DIR + name).mkdirs();
		}

		// This program operates in meters. It will convert all input variables from
		// inches to meters.
		for (Waypoint point : points) {
			point.x *= 0.0254;
			point.y *= 0.0254;
		}

		// Check if trajectory is already there and updated
		boolean match = true;
		if (new File(SAVE_DIR + name + "/waypoints.csv").exists()) {
			try {
				BufferedReader fileReader = new BufferedReader(
						new FileReader(new File(SAVE_DIR + name + "/waypoints.csv")));
				String ln;
				int i = 0;
				while ((ln = fileReader.readLine()) != null) {
					String[] vals = ln.split(",");
					try {
						Double.parseDouble(vals[0]);
						Double.parseDouble(vals[1]);
						Double.parseDouble(vals[2]);
					} catch (Exception e) {
						continue;
					}
					if (vals.length == 3) {
						double x = new Double(vals[0]);
						double y = new Double(vals[1]);
						double ang = new Double(vals[2]);
						if (x == points[i].x && y == points[i].y && ang == points[i].angle) {
							i++;
						} else {
							match = false;
							break;
						}
					}
				}
				fileReader.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		if (match && !forceRegen) {
			return null;
		}
		System.out.println("Updating trajectory for " + name);
		Thread calculationThread = new Thread() {
			public void run() {
				BufferedWriter pathWriter;
				BufferedWriter wayptWriter;

				Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_QUINTIC, Config.SAMPLES_HIGH,
						TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
				Trajectory trajectory = Pathfinder.generate(points, config);
				TankModifier modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);

				File leftTrajectoryFile = new File(SAVE_DIR + name + "/LeftTrajectory.traj");
				Pathfinder.writeToFile(leftTrajectoryFile, modifier.getLeftTrajectory());

				File rightTrajectoryFile = new File(SAVE_DIR + name + "/RightTrajectory.traj");
				Pathfinder.writeToFile(rightTrajectoryFile, modifier.getRightTrajectory());

				try {
					pathWriter = new BufferedWriter(new FileWriter(SAVE_DIR + name + "/paths.csv"));
					pathWriter.write("x_left,y_left,x_right,y_right");
					pathWriter.newLine();
					for (int i = 0; i < modifier.getLeftTrajectory().length()
							&& i < modifier.getRightTrajectory().length(); i++) {
						Trajectory.Segment segl = modifier.getLeftTrajectory().get(i);
						Trajectory.Segment segr = modifier.getRightTrajectory().get(i);
						pathWriter.write(segl.x + "," + segl.y + "," + segr.x + "," + segr.y);
						pathWriter.newLine();
					}
					pathWriter.close();

					wayptWriter = new BufferedWriter(new FileWriter(SAVE_DIR + name + "/waypoints.csv"));
					wayptWriter.write("x,y,angle");
					wayptWriter.newLine();
					for (Waypoint point : points) {
						wayptWriter.write(point.x + "," + point.y + "," + point.angle);
						wayptWriter.newLine();
					}
					wayptWriter.close();

				} catch (IOException e) {
					e.printStackTrace();
				}

				System.out.println("Trajectory Updated: " + name);
			}
		};

		calculationThread.start();
		return calculationThread;
	}
}
