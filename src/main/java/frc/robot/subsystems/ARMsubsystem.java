// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class ARMsubsystem extends SubsystemBase {
  /** Creates a new ARMsubsystem. */
  WPI_TalonFX motorM21 = new WPI_TalonFX(21);
  DigitalInput limitswhic = new DigitalInput(1);
  boolean stateARM;
  boolean stateLVLARM;
  Orchestra music;

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
    motorM21.configMotionCruiseVelocity(20000);
    motorM21.configMotionAcceleration(10000);

    motorM21.setSensorPhase(true);

    List<TalonFX> or = new ArrayList<TalonFX>();
    or.add(motorM21);

    music = new Orchestra(or, "src\\main\\deploy\\gg.chrp");
    music.play();
    //state
    stateARM = true;
    stateLVLARM = true;

  }

  @Override
  public void periodic() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shuffleboard");
    SmartDashboard.putNumber("angalARM", motorM21.getSelectedSensorPosition());
    SmartDashboard.putBoolean("lvl 2/3", this.getstatelvl());
    SmartDashboard.putBoolean("limitsvhic", limitswhic());
    NetworkTableValue.makeBoolean(stateLVLARM);
    motorM21.getActiveTrajectoryPosition();
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
  public boolean getstatelvl() {
    return this.stateLVLARM;
  }

  //Chang state ARM
  public void changstate() {
    this.stateARM = !stateARM;
  }

  public void setstate(Boolean tuful) {
    this.stateARM = tuful;
  }

  public void changstatelvl() {
    this.stateLVLARM = !stateLVLARM;
  }

  public void setstatelvl(Boolean tuful) {
    this.stateLVLARM = tuful;
  }

  //test (delete bifor start the תחרות)
  public void set(double speed) {
    motorM21.set(speed);
  }
  
  public void startmusic() {
    music.play();
  }

  //limitswhic
  public boolean limitswhic() {
    return limitswhic.get();
  }
}