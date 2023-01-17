// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class autoPathPlanner extends SequentialCommandGroup {
    public autoPathPlanner(Swerve s_Swerve) {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("tryPath", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(0, 0.0, 0);
        PIDController yController = new PIDController(0, 0.0, 0);
        PIDController zController = new PIDController(0, 0.0, 0);

        PPSwerveControllerCommand ppPath = new PPSwerveControllerCommand(
                examplePath, 
                s_Swerve::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                yController, // Y controller (usually the same values as X controller)
                zController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                s_Swerve::setModuleStates, // Module states consumer
                s_Swerve);

        new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialHolonomicPose())),
            ppPath,
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }
}