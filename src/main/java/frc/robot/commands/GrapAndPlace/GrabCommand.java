// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrapAndPlace;
import frc.robot.subsystems.Grappersubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabCommand extends CommandBase {
  /** Creates a new GrabCommand. */
  private Grappersubsystem m_Grapper;
  public GrabCommand(Grappersubsystem grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Grapper = grapper;
    addRequirements(m_Grapper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(true) {
      if (m_Grapper.getstate()) {
        m_Grapper.speed(-0.80);
      }
      else {
        m_Grapper.speed(80);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
