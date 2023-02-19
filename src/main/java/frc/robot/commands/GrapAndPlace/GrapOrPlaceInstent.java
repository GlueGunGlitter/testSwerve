// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrapOrPlaceInstent extends InstantCommand {
  private Grappersubsystem m_Grapper;
  private ARMsubsystem m_ARM;
  public GrapOrPlaceInstent(ARMsubsystem ARM, Grappersubsystem grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Grapper = grapper;
    addRequirements(m_Grapper);
    m_ARM = ARM;
    addRequirements(m_ARM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ARM.getstate()) {
      if(m_Grapper.getstate()) {
        m_Grapper.set(-0.8);
      }
      else {
        m_Grapper.set(-0.8);
      }
    }
    else {
      if(m_Grapper.getstate()) {
        m_Grapper.set(0.8);
      }
      else {
        m_Grapper.set(0.8);
      }
    }
  }
}
