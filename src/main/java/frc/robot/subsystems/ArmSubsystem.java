// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ARMsubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM21 = new WPI_TalonFX(21);

  public ARMsubsystem() {
    //config motorM21
    motorM21.configFactoryDefault();
    motorM21.setNeutralMode(NeutralMode.Brake);
    motorM21.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor ,0 ,0);

    //SensorPosition get
    motorM21.setSelectedSensorPosition(0);

    //limit motorM21
    motorM21.configForwardSoftLimitThreshold(300000, 30);
    motorM21.configReverseSoftLimitThreshold(300000, 30);
    motorM21.configForwardSoftLimitEnable(true, 0);
    motorM21.configReverseSoftLimitEnable(true, 0);

    //Deadband
    motorM21.configNeutralDeadband(0.1);
    
  
    //PIDmotor
    motorM21.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 30);
    motorM21.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);
    motorM21.config_kP(0, 0.0, 30);
    motorM21.config_kI(0, 0.0, 30);
    motorM21.config_kD(0, 0.0, 30);
    motorM21.config_kF(0, 0.0, 30);

    //PIDSpeed/POWER
    motorM21.configMotionCruiseVelocity(20000);
    motorM21.configMotionAcceleration(10000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorM21.getSelectedSensorPosition();
    ShuffleboardTab tab = Shuffleboard.getTab("Shuffleboard");
    SmartDashboard.putNumber("towerX", motorM21.getSelectedSensorPosition());
  }

  //SensorPosition
  public double getSensorPosition() {
    return motorM21.getSelectedSensorPosition();
  }

  //set SensorPosition
  public void setSensorPosition(int nan) {
    motorM21.setSelectedSensorPosition(nan);
  }

  //stopmotorM21
  public void stopARM() {
    motorM21.set(0);
  }
  
  //mousion magic setPosision
  public void setposison(double Pos) {
    double position = Pos * 1;
    motorM21.set(TalonFXControlMode.MotionMagic, position);
  }
  
  //get SensorPosition PID
  public double getposison() {
    double curretAngle = motorM21.getSelectedSensorPosition(0)/ 1;
    return curretAngle;
  }
}
