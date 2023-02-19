// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;



public class Grappersubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM19 = new WPI_TalonFX(19);
  boolean stateGrapper;
  
  AnalogInput m_Ultrasonic = new AnalogInput(3);

  
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
    SmartDashboard.putBoolean("KON", this.getstate());
    SmartDashboard.putBoolean("KUB", !this.getstate());

    if (m_Ultrasonic.getAverageValue() > 180) { //kub
      setstate(false);
    }
    else if (m_Ultrasonic.getAverageValue() > 310) {
      setstate(true); 
    }

  }

  //Stop motorM19
  public void StopGrapper() {
    motorM19.set(0);
  }

  public double getdistans() {
    return m_Ultrasonic.getAverageValue();
  }

  public void set(double speed) {
    motorM19.set(speed);
  }

  public boolean getstate() {
    return this.stateGrapper;
  }

  public void changstate() {
    this.stateGrapper = !stateGrapper;
  }

  public void setstate(Boolean tuful) {
    this.stateGrapper = tuful;
  }
}

