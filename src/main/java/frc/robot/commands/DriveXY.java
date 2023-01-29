// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.util;
import frc.robot.Constants;
import frc.robot.subsystems.Swervesubsystem;
import frc.robot.subsystems.limelightSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveXY extends SequentialCommandGroup {
  /** Creates a new DriveXY. */
  Swervesubsystem m_swerve;
  private limelightSubSystem m_Limelight;
  public DriveXY(Swervesubsystem swerve, limelightSubSystem light) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    addRequirements(m_swerve);
    m_Limelight = light;
    addRequirements(m_Limelight);
    
    double y = -util.distanceFromTarget(m_Limelight.targetY());

    PathPlannerTrajectory traj2 = PathPlanner.generatePath(
    new PathConstraints(4, 3), 
      List.of(new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), new PathPoint(new Translation2d(0, y), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))) // position, heading(direction of travel), holonomic rotation
    );

    PIDController xController = new PIDController(0.0, 0.0, 0);
    PIDController yController = new PIDController(0.0, 0.0, 0);
    PIDController zController = new PIDController(0.0, 0.0, 0);


    addCommands(new InstantCommand(() -> m_swerve.zeroGyro()));
    addCommands(new InstantCommand(() -> m_swerve.resetModulesToAbsolute()));
    addCommands(
      new PPSwerveControllerCommand(
        traj2, 
        m_swerve::getPose, // Pose supplier
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        yController, // Y controller (usually the same values as X controller)
        zController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        m_swerve::setModuleStates, // Module states consumer
        m_swerve)
    );
  }
}
