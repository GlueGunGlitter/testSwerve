// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrapOrPlace extends InstantCommand {
  private ARMsubsystem m_ARM;
  private Grappersubsystem m_Grapper;
  public GrapOrPlace(ARMsubsystem ARM ,Grappersubsystem Grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Grapper = Grapper;
    addRequirements(m_Grapper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ARM.getstate()) {
      if(m_Grapper.getstate()) {
        m_Grapper.speed(-0.8);
      }
      else {
        m_Grapper.speed(0.8);
      }
    }
    else {
      if(m_Grapper.getstate()) {
        m_Grapper.speed(0.8);
      }
      else {
        m_Grapper.speed(-0.8);
      }
    }
  }
}
