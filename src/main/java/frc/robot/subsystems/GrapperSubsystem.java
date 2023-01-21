// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrapperSubsystem extends SubsystemBase {
  /** Creates a new GrapperSubsystem. */
  TalonFX grap = new TalonFX(19);

  public GrapperSubsystem() {
    grap.configFactoryDefault();
    grap.setNeutralMode(NeutralMode.Brake);

  }

  public void grapGamePiece() {
    grap.set(TalonFXControlMode.PercentOutput, 0.5);
  }

  public void grapStop() {
    grap.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void grapDropGamePiece() {
    grap.set(TalonFXControlMode.PercentOutput, -0.5);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
