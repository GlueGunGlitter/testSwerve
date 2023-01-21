// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX arm = new TalonFX(21);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    arm.configFactoryDefault();
    arm.setNeutralMode(NeutralMode.Brake);
  }

  public void moveArmUp() {
    arm.set(TalonFXControlMode.PercentOutput, 0.3);
  }

  public void moveArmDown() {
    arm.set(TalonFXControlMode.PercentOutput, -0.3);
  }

  public void stopArm() {
    arm.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
