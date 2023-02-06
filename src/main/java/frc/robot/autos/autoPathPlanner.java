// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ARMsubsystem;
import frc.robot.subsystems.Swervesubsystem;

/** Add your docs here. */
public class autoPathPlanner extends SequentialCommandGroup {
    public autoPathPlanner(Swervesubsystem s_Swerve, ARMsubsystem m_arm) {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("BetterWork", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0.0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0.0, 0);
        PIDController zController = new PIDController(0, 0.0, 0);

        SequentialCommandGroup arm = new SequentialCommandGroup(
            new InstantCommand(() -> m_arm.setposison(1)),
            new WaitCommand(1.5),
            new InstantCommand(() -> m_arm.setposison(0)),
            new WaitCommand(1),
            new InstantCommand(() -> m_arm.stopARM())
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("upArm", arm);


        PPSwerveControllerCommand ppPath = new PPSwerveControllerCommand(
                examplePath, 
                s_Swerve::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                yController, // Y controller (usually the same values as X controller)
                zController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                s_Swerve::setModuleStates, // Module states consumer
                s_Swerve);

        FollowPathWithEvents follow = new FollowPathWithEvents(ppPath, examplePath.getMarkers(), eventMap);

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialHolonomicPose())),
            ppPath,
            new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }
}
