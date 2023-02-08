// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class GrapOrPlace extends CommandBase {
  /** Creates a new GrapOrPlace. */
  private Grappersubsystem m_Grapper;
  private ARMsubsystem m_ARM;

  public GrapOrPlace(ARMsubsystem ARM, Grappersubsystem grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Grapper = grapper;
    addRequirements(m_Grapper);
    m_ARM = ARM;
    addRequirements(m_ARM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(m_Grapper.getProximity() > 1 && m_Grapper.getProximity() < 2) {
    //   m_Grapper.setstate(true);
    // }
    // if (m_Grapper.getProximity() > 3 && m_Grapper.getProximity() < 4) {
    //   m_Grapper.setstate(false);
    // }

    if(m_ARM.getstate()) {
      if(m_Grapper.getstate()) {
        m_Grapper.speed(-0.8);
      }
      else {
        m_Grapper.speed(-0.8);
      }
    }
    else {
      if(m_Grapper.getstate()) {
        m_Grapper.speed(0.8);
      }
      else {
        m_Grapper.speed(0.8);
      }
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