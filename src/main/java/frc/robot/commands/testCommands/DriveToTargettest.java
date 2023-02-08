// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTargettest extends SequentialCommandGroup {
  /** Creates a new DriveToTarget2. */
  private limelightSubSystem m_Limelight;
  private Swervesubsystem m_Swerve;

  public DriveToTargettest(limelightSubSystem light ,Swervesubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Limelight = light;
    addRequirements(m_Limelight);
    m_Swerve = swerve;
    addRequirements(m_Swerve);

    final double DESIRED_TARGET_AREA = 0.95;

    Pose2d pose = swerve.getPose();

    double steer_value = m_Limelight.targetX() * 0.03;
    double drive_value =  0.6 + (m_Limelight.targetArea() - 0.006) * (0.2 - 0.6) / (0.95 - 0.006);
    double rotate_value = pose.getRotation().getDegrees() * 0.6 / 180;
    

    addCommands(new TeleopSwerveCommand(swerve, () -> drive_value, () -> 0, () -> 0, () -> false, () -> false).until(() -> m_Limelight.targetArea() > DESIRED_TARGET_AREA));
    
  }
}
