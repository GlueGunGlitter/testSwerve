// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTargetSet extends SequentialCommandGroup {
  /** Creates a new DriveToTarget. */
  private limelightSubSystem m_Limelight;
  private Swervesubsystem m_Swerve;
  public DriveToTargetSet(limelightSubSystem light, Swervesubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Limelight = light;
    addRequirements(m_Limelight);
    m_Swerve = swerve;
    addRequirements(m_Swerve);
    addCommands(new INTDriveToTargetXY(swerve, light, 0, 0));
  }
}
