// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SetARMpostionToPlace extends CommandBase {
  /** Creates a new SetARMpostionToPlace. */
  private ARMsubsystem m_ARM;
  public SetARMpostionToPlace(ARMsubsystem ARM, Grappersubsystem Grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_ARM.getstatelvl()) {      
     if (m_ARM.getstate()) {
       m_ARM.setposison(69);
       m_ARM.changstate();
    }
    else {
       m_ARM.setposison(10);
       m_ARM.changstate();
     }
    }
    else {
      if (m_ARM.getstate()) {
        m_ARM.setposison(75);
        m_ARM.changstate();
     }
     else {
        m_ARM.setposison(10);
        m_ARM.changstate();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ARM.getSensorPosition() > 50000;
  }
}
