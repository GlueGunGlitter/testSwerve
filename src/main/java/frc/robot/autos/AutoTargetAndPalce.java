// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTargetAndPalce extends SequentialCommandGroup {
  /** Creates a new AutoTargetAndPalce. */
  private Grappersubsystem m_Grapper;
  private ARMsubsystem m_ARM;
  private Swervesubsystem m_Swerve;
  private limelightSubSystem m_Light;
  public AutoTargetAndPalce(Grappersubsystem grapper ,ARMsubsystem ARM ,Swervesubsystem swerve , limelightSubSystem light) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Grapper = grapper;
    addRequirements(m_Grapper);
    m_Swerve = swerve;
    addRequirements(m_Swerve);
    m_Light = light;
    addRequirements(m_Light);
    addCommands(new DriveToTarget(light, swerve));
    addCommands(new placeCommand(light, ARM, grapper));
  }
}
