// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Grappersubsystem extends SubsystemBase {
  /** Creates a new Grapper. */
  WPI_TalonFX motorM19 = new WPI_TalonFX(19);
  boolean stateGrapper;

  public Grappersubsystem() {
    //reset config motorM19
    motorM19.configFactoryDefault();
    motorM19.setNeutralMode(NeutralMode.Brake);

    stateGrapper = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IN KON / AUT KUB", this.getstate());
    SmartDashboard.putBoolean("AUT KON / IN KUB", !this.getstate());
  }

  //Stop motorM19
  public void StopGrapper() {
    motorM19.set(0);
  }
  
  //Close Grap
  public void GrapORRelis(double speed) {
    motorM19.set(speed); //cub - to relis for cone + to relis//

  }

  public void speed(double speed) {
    motorM19.set(speed);
  }

  public boolean getstate() {
    return this.stateGrapper;
  }

  public void changstate() {
    this.stateGrapper = !stateGrapper;
  }
}
