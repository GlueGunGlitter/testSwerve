// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;



public class Grappersubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM19 = new WPI_TalonFX(19);
  boolean stateGrapper;
  ColorMatch m_colorMatcher = new ColorMatch();

  // color sensor
  Color kBlankTarget = new Color(0,0,0);
  Color kPurpleTarget = new Color(128,0,128);
  Color kYellowTarget = new Color(255,255,0);

  //color sensor
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  int proximity = m_colorSensor.getProximity();


  public Grappersubsystem() {
    //reset config motorM19
    motorM19.configFactoryDefault();
    motorM19.setNeutralMode(NeutralMode.Brake);

    stateGrapper = true;
  }

  
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Proximity", getProximity());
    SmartDashboard.putBoolean("IN KON / AUT KUB", this.getstate());
    SmartDashboard.putBoolean("AUT KON / IN KUB", !this.getstate());
  }

  //Stop motorM19
  public void StopGrapper() {
    motorM19.set(0);
  }

  public int getProximity() {
    return m_colorSensor.getProximity();
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

