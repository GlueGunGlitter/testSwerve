// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.lib.util.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTargettest extends SequentialCommandGroup {
  /** Creates a new DriveToTarget2. */
  private limelightSubSystem m_Limelight;
  private Swervesubsystem m_Swerve;

  public DriveToTargettest(limelightSubSystem light ,Swervesubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Limelight = light;
    addRequirements(m_Limelight);
    m_Swerve = swerve;
    addRequirements(m_Swerve);

    double y = util.kalculatdisrtans(m_Limelight.targetX());
    
    addCommands(new INTDriveToTargetXY(swerve, 0,  1));
    
  }
}

