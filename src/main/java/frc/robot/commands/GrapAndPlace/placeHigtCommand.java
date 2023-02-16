// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class placeHigtCommand extends CommandBase {
  /** Creates a new placeHigtCommand. */
  private ARMsubsystem m_ARM;

  public placeHigtCommand(ARMsubsystem ARM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_ARM.setposison(65);
      m_ARM.setstate(false);
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ARM.getSensorPosition() < 65;
  }
}
