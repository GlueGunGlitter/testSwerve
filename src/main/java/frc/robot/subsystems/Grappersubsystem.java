// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CIEColor;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;



public class Grappersubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM19 = new WPI_TalonFX(19);
  boolean stateGrapper;
  ColorMatch m_colorMatcher = new ColorMatch();
  int color;

  
  // color sensor
  Color kBlankTarget = new Color(0,0,0);
  Color kPurpleTarget = new Color(128,0,128);
  Color kYellowTarget = new Color(255,255,0);

  //color sensor
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  int proximity = m_colorSensor.getProximity();
  int getBlue = m_colorSensor.getBlue() -250;
  int getGreen = m_colorSensor.getGreen() -400;
  int getRed = m_colorSensor.getRed() -200;
  double getIR = m_colorSensor.getIR();

  
  public Grappersubsystem() {
    //reset config motorM19
    motorM19.configFactoryDefault();
    motorM19.setNeutralMode(NeutralMode.Brake);

    stateGrapper = true; //kon
  }

  
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    SmartDashboard.putNumber("getBlue", m_colorSensor.getBlue());
    SmartDashboard.putNumber("getGreen", m_colorSensor.getGreen());
    SmartDashboard.putNumber("getRed", m_colorSensor.getRed());
    SmartDashboard.putNumber("getIR", m_colorSensor.getIR());
    SmartDashboard.putBoolean("KON", this.getstate());
    SmartDashboard.putBoolean("KUB", !this.getstate());
    SmartDashboard.putNumber("color", this.getint());

    if (m_colorSensor.getBlue() > 180) { //kub
      setstate(false);
    }
    else if (m_colorSensor.getGreen() > 310 && m_colorSensor.getRed() > 170) {
      setstate(true); 
    }

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


  public int getint() {
    return color;
  }


  public void changstate() {
    this.stateGrapper = !stateGrapper;
  }

  public void setstate(Boolean tuful) {
    this.stateGrapper = tuful;
  }

  public void getGreen() {
    m_colorSensor.getGreen();
  }

  public void getRed() {
    m_colorSensor.getRed();
  }
}

