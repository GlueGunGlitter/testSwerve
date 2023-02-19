// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ARMsubsystem;

public class ResetARM extends CommandBase {
  /** Creates a new ResetArm. */
  ARMsubsystem m_arm;
  public ResetARM(ARMsubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.set(-0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.set(0);
    m_arm.setSensorPosition(0);
    m_arm.setposison(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.limitswhic();
  }
}
