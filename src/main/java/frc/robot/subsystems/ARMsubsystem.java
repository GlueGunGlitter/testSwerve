// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ARMsubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM0 = new WPI_TalonFX(0);

  public ARMsubsystem() {
    //config motorM0
    motorM0.configFactoryDefault();
    motorM0.setNeutralMode(NeutralMode.Brake);
    motorM0.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor ,0 ,0);

    //SensorPosition get
    motorM0.setSelectedSensorPosition(0);

    //limit motorM0
    motorM0.configForwardLimitSwitchSource(null, null);
    motorM0.configReverseLimitSwitchSource(null, null);
    motorM0.configForwardSoftLimitEnable(true, 0);
    motorM0.configReverseSoftLimitEnable(true, 0);

    //Deadband
    motorM0.configNeutralDeadband(0.1);
    
  
    //PIDmotor
    motorM0.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 30);
    motorM0.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 30);
    motorM0.config_kP(0, 0.0, 0);
    motorM0.config_kI(0, 0.0, 0);
    motorM0.config_kD(0, 0.0, 0);
    motorM0.config_kF(0, 0.0, 0);

    //PIDSpeed/POWER
    motorM0.configMotionCruiseVelocity(20000);
    motorM0.configMotionAcceleration(10000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorM0.getSelectedSensorPosition();
  }

  //SensorPosition
  public double getSensorPosition() {
    return motorM0.getSelectedSensorPosition();
  }

  //set SensorPosition
  public void setSensorPosition(int nan) {
    motorM0.setSelectedSensorPosition(nan);
  }

  //stopmotorM0
  public void stopARM() {
    motorM0.set(0);
  }
  
  //mousion magic setPosision
  public void setposison(double Pos) {
    double position = Pos * 1;
    motorM0.set(TalonFXControlMode.MotionMagic, position);
  }
  
  //get SensorPosition PID
  public double getposison() {
    double curretAngle = motorM0.getSelectedSensorPosition(0)/ 1;
    return curretAngle;
  }
}
