// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;



public class Grappersubsystem extends SubsystemBase {
  Color kBlankTarget = new Color(0,0,0);
  Color kPurpleTarget = new Color(128,0,128);
  Color kYellowTarget = new Color(255,255,0);
  
 
  /** Creates a new Grapper. */
  WPI_TalonFX motorM0 = new WPI_TalonFX(19);
  ColorMatch m_colorMatcher = new ColorMatch();

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  int proximity = m_colorSensor.getProximity();

  public Grappersubsystem() {
    //reset config motorM0
    motorM0.configFactoryDefault();
    motorM0.setNeutralMode(NeutralMode.Brake);

  }

  
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic() {
      
    SmartDashboard.putNumber("Proximity", getProximity());

  }



  public int getProximity() {
    return m_colorSensor.getProximity();
  }
  //Stop motorM0
  public void StopGrapper() {
    motorM0.set(0);
  }
  
  //Close Grap


  public void CloseGrap(double speed) {
    motorM0.set(speed);
  }


  //Relese Grap
  public void ReleseGrap(double speed) {
    motorM0.set(speed);
  }
}

