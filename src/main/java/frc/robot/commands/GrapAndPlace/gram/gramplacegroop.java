// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace.gram;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class gramplacegroop extends SequentialCommandGroup {
  /** Creates a new gramplacegroop. */
  GramSubsystem m_gram;
  public gramplacegroop(GramSubsystem gram) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new gramgrapCommand(gram));
    addCommands(new gramplaceCommand(gram));
  }
}
