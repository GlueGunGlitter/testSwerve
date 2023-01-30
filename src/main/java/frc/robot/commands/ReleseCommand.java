// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grappersubsystem;
import frc.robot.subsystems.ARMsubsystem;
public class ReleseCommand extends CommandBase {
  /** Creates a new ReleseCommand. */
  private Grappersubsystem m_Grapper;
  private ARMsubsystem m_ARM;

  public ReleseCommand(Grappersubsystem grapper , ARMsubsystem ARM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Grapper = grapper;
    addRequirements(m_Grapper);
    m_ARM = ARM;
    addRequirements(m_ARM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_ARM.getSensorPosition() == 1) {
      m_Grapper.ReleseGrap(1);
    }
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
    return false;
  }
}
