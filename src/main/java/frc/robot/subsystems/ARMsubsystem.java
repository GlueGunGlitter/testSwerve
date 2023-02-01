// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
  Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  boolean stateARM;

  public ARMsubsystem() {
    //config motorM21
    motorM21.configFactoryDefault();
    motorM21.setNeutralMode(NeutralMode.Brake);

    //SensorPosition get
    motorM21.setSelectedSensorPosition(0);
    motorM21.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor ,0 ,0);

    //limit motorM21
    motorM21.configForwardSoftLimitThreshold(85000, 30);
    motorM21.configForwardSoftLimitEnable(true, 6000);

    //Deadband
    motorM21.configNeutralDeadband(0.1);
  
    //PIDmotor
    motorM21.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 30);
    motorM21.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);
    motorM21.config_kP(0, 0.25, 30); //0.25
    motorM21.config_kI(0, 0.00005, 30); //0.000018
    motorM21.config_kD(0, 0.0, 30); //0.001
    motorM21.config_kF(0, 0.0, 30); //0.0

    //PIDSpeed/POWER
    motorM21.configMotionCruiseVelocity(5000);
    motorM21.configMotionAcceleration(2500);

    motorM21.setSensorPhase(true);

    //state
    stateARM = true;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angal", motorM21.getSelectedSensorPosition());
    SmartDashboard.putNumber("encoder", encoder.getRaw());
    SmartDashboard.putString("control", motorM21.getControlMode().toString());
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
    double position = Pos * 1000;
    motorM21.set(TalonFXControlMode.MotionMagic, position);
  }
  
  //get SensorPosition PID
  public double getposison() {
    double curretAngle = motorM21.getSelectedSensorPosition()/ 1000;
    return curretAngle;
  }

  //get state ARM
  public boolean getstate() {
    return this.stateARM;
  }

  //Chang state ARM
  public void changstate() {
    this.stateARM = !stateARM;
  }

  //test (delete bifor start the תחרות)
  public void set(double speed) {
    motorM21.set(speed);
  }
}