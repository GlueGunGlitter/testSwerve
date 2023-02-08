// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swervesubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetDriveToTargetXY extends SequentialCommandGroup {

  public SetDriveToTargetXY(Swervesubsystem m_Swerve, double x, double y){

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
       
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = util.getTraj(config, x, y, m_Swerve.getPose());

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.aPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.aPXController, 0, 0),
            new PIDController(Constants.AutoConstants.aPYController, 0, 0),
            thetaController,
            m_Swerve::setModuleStates,
            m_Swerve);


    addCommands(
        swerveControllerCommand
    );
}
}
