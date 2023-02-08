// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class placeCommandGroop extends SequentialCommandGroup {
  /** Creates a new place. */
  private ARMsubsystem m_ARM;
  private Grappersubsystem m_Grapper;

  public placeCommandGroop(ARMsubsystem ARM , Grappersubsystem grapper ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Grapper = grapper;
    addRequirements(m_Grapper);


    addCommands(new SetARMpostionToPlace(ARM, grapper));
    addCommands(new GrapOrPlace(ARM, grapper));
    addCommands(Commands.run(()->m_Grapper.StopGrapper()));
    // addCommands(new SetARMpostionToPlace(ARM));
  }
}