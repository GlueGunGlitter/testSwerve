// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ARMsubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetARMpotionToPlace extends InstantCommand {
  private ARMsubsystem m_ARM;
  public SetARMpotionToPlace(ARMsubsystem ARM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ARM = ARM;
    addRequirements(m_ARM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_ARM.getstate()) {
      m_ARM.setposison(80);
      m_ARM.changstate();
   }
   else {
      m_ARM.setposison(10);
      m_ARM.changstate();
    }
  }
}
