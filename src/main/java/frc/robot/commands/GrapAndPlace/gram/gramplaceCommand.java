// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace.gram;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class gramplaceCommand extends CommandBase {
  /** Creates a new gramgrapCommand. */
  GramSubsystem m_gram;
  Timer timer;
  public gramplaceCommand(GramSubsystem gram) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gram = gram;
    addRequirements(m_gram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    m_gram.set(-0.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gram.StopGram();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2;
  }
}
