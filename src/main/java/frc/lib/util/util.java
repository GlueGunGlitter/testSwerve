// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class util {
    public static Trajectory getTraj(TrajectoryConfig config , double x , double y , Pose2d initPose) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        initPose,
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d( x, y )),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(initPose.getX() + x , initPose.getY() + y , new Rotation2d(0)),
        config);

        return traj;

    }
}
