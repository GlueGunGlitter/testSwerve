// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swervesubsystem;

public class StableCommand extends CommandBase {
  Swervesubsystem m_swerve;
  /** Creates a new StableCommand. */
  public StableCommand(Swervesubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
