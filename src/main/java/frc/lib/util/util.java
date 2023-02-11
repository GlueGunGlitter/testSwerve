// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.lib.math.*;

/** Add your docs here. */
public class util {

    
    public static Trajectory getTraj(TrajectoryConfig config , double x , double y , Pose2d initPose) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(initPose.getX(), initPose.getY(), new Rotation2d(0)) ,
        List.of(new Translation2d(0, 0), new Translation2d( x, y )),
        new Pose2d(initPose.getX() + x , initPose.getY() + y , new Rotation2d(0)),
        config);

        return traj;

    }

    public static double distanceFromTarget(double targetOffsetAngle_Vertical) {
        double limelightMountAngleDegrees = 10.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = Conversions.CmToInch(52.2);

        // distance from the target to the floor
        double goalHeightInches = Conversions.CmToInch(60);

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

        return Conversions.InchToM(distanceFromLimelightToGoalInches);
    }
}
