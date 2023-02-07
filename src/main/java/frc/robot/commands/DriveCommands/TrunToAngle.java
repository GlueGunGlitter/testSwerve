// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swervesubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrunToAngle extends PIDCommand {
  /** Creates a new TrunToAngle. */
  
  public TrunToAngle(Swervesubsystem m_swerve, double angleTurn) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD),
        // This should return the measurement
        m_swerve::getYawDouble,
        // This should return the setpoint (can also be a constant)
        angleTurn,
        // This uses the output
        output -> m_swerve.drive(new Translation2d(0, 0), output, false, true),
        m_swerve);
        
        getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
