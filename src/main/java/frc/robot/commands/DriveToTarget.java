// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.DriveXYZ;
import frc.robot.subsystems.limelightSubSystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTarget extends ParallelDeadlineGroup {
  /** Creates a new DriveToTarget. */
  private limelightSubSystem m_light;
  public DriveToTarget(limelightSubSystem light) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new InstantCommand());
    m_light = light;
    addRequirements(m_light);

    addCommands(null);
    // addCommands(new FooCommand(), new BarCommand());
  }
}