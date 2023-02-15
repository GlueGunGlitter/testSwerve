// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automezation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class DriveToTragetArea extends CommandBase {

  Swervesubsystem swerve;
  limelightSubSystem light;
  Grappersubsystem m_grapper;
  double DESIRED_TARGET_AREA;

  public DriveToTragetArea(Swervesubsystem m_swerve, limelightSubSystem m_limelight, Grappersubsystem grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;
    addRequirements(swerve);
    light = m_limelight;
    addRequirements(light);
    m_grapper = grapper;
    addRequirements(m_grapper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    if (m_grapper.getstate()) {
    light.setpipline(2);
    }
    else if (!m_grapper.getstate()) {
    light.setpipline(3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = swerve.getPose();

    if (m_grapper.getstate()) {
      DESIRED_TARGET_AREA = 0.45; //kon
    }    
    else {
      DESIRED_TARGET_AREA = 8; //kub
    }

    PIDController steerController = new PIDController(0.012, 0, 0.0);
    steerController.enableContinuousInput(-27, 27);
    double steer_value = MathUtil.clamp(steerController.calculate(light.targetX(), 0), -0.4, 0.4);

    PIDController driveController = new PIDController(0.0, 0, 0);
    double drive_value = MathUtil.clamp(driveController.calculate(light.targetArea(), DESIRED_TARGET_AREA), 0.4, 0);

    PIDController rotateController = new PIDController(0.01, 0, 0);
    rotateController.enableContinuousInput(-180, 180);
    double rotate_value = MathUtil.clamp(rotateController.calculate(pose.getRotation().getDegrees(), 0), -0.1, 0.1);
  

    // if (rotate_value < 1 || rotate_value > -1) {
    //  steer_value = -light.targetX() * 0.022;
    //  drive_value =  0.4 + (light.targetArea() - 0.006) * (0.1 - 0.4) / (0.95 - 0.006);
    // }
    // else {
    //  steer_value = -light.targetX() * 0.025;
    //  drive_value =  0.1;
    // }

    swerve.drive(
            new Translation2d(drive_value, steer_value).times(Constants.Swerve.maxSpeed), 
            rotate_value * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
  }

  // Called once the command ends or i0.4 + (light.targetArea() - 0.006) s interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      true, 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return light.targetArea() > DESIRED_TARGET_AREA;
  }

  public double modAngle(double value) {
    return ((value + 180) % 360) - 180;
  }
}
