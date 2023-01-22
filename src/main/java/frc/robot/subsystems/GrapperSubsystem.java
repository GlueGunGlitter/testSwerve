// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Grappersubsystem extends SubsystemBase {
  /** Creates a new Grapper. */
  WPI_TalonFX motorM0 = new WPI_TalonFX(0);
  public Grappersubsystem() {
    //reset config motorM0
    motorM0.configFactoryDefault();
    motorM0.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Stop motorM0
  public void StopGrapper() {
    motorM0.set(0);
  }
  
  //Close Grap
  public void CloseGrap() {
    motorM0.set(1);
  }

  //Relese Grap
  public void ReleseGrap() {
    motorM0.set(1);
  }
}