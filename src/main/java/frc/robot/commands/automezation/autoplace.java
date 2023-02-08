// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automezation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GrapAndPlace.SetARMpostionToPlace;
import frc.robot.commands.GrapAndPlace.placeCommandGroop;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoplace extends SequentialCommandGroup {
  /** Creates a new autoplace. */
  private Swervesubsystem m_swerve;
  private limelightSubSystem m_light;
  private ARMsubsystem m_ARM;
  private Grappersubsystem m_Grapper;
  public autoplace(Swervesubsystem swerve, limelightSubSystem limelight, ARMsubsystem ARM ,Grappersubsystem Grapper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    addRequirements(m_swerve);
    m_light = limelight;
    addRequirements(m_light);
    m_ARM = ARM;
    addRequirements(m_ARM);
    m_Grapper = Grapper;
    addRequirements(m_Grapper);

    addCommands(new DriveToTragetArea(swerve, limelight));
    addCommands(new SetARMpostionToPlace(ARM));
    addCommands(Commands.runOnce(()->m_Grapper.speed(8)));
    addCommands(new WaitCommand(1.5));
    addCommands(Commands.runOnce(()->m_Grapper.StopGrapper()));
    addCommands(new SetARMpostionToPlace(ARM));
    
  }
}
