// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class placeHigtCommand extends CommandBase {
  /** Creates a new placeHigtCommand. */
  private ARMsubsystem m_ARM;
  private GramSubsystem m_gram;
  private Timer timer;
  
  public placeHigtCommand(ARMsubsystem ARM, GramSubsystem gram) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_gram = gram;
    addRequirements(m_gram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    if (m_gram.getbool()) {
      m_gram.setposison(0);
    }
    else {
      m_ARM.setposison(65);
      m_ARM.setstate(false);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ARM.getSensorPosition() < 65 && timer.get() > 2;
  }
}
