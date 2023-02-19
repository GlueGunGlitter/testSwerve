// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.examination;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class gramExamination extends CommandBase {
  /** Creates a new gramExamination. */
  GramSubsystem m_gram;
  public gramExamination(GramSubsystem gram) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gram = gram;
    addRequirements(m_gram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_gram.getbool()) {
      m_gram.setposison(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gram.setbool(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gram.getposison() <= 0;
  }
}