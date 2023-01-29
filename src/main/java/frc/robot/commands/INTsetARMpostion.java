// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class INTsetARMpostion extends CommandBase {
  private double PosAngal;
  /** Creates a new SetARMPostionToPlace. */
  private limelightSubSystem m_Light;
  private ARMsubsystem m_ARM;

  public INTsetARMpostion(ARMsubsystem ARM ,limelightSubSystem Light , int i) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Light = Light;
    addRequirements(m_Light);
  }

  public INTsetARMpostion(ARMsubsystem m_arm2, double d) {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ARM.setposison(PosAngal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
