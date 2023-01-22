// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.sparse.csc.misc.ImplCommonOpsWithSemiRing_DSCC;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.limelightSubSystem;
import frc.robot.subsystems.ARMsubsystem;
import frc.robot.subsystems.Grappersubsystem;
import frc.robot.commands.ReleseCommand;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.SetARMPostionToPlace;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class place extends SequentialCommandGroup {
  /** Creates a new place. */
  private limelightSubSystem m_light;
  private ARMsubsystem m_ARM;
  private Grappersubsystem m_Grapper;

  public place(limelightSubSystem light ,ARMsubsystem ARM , Grappersubsystem grapper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_light = light;
    addRequirements(m_light);
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Grapper = grapper;
    addRequirements(m_Grapper);

    addCommands();
    new DriveToTarget(light);
    new SetARMPostionToPlace(ARM, light);
    new ReleseCommand(grapper, ARM);
  }
}