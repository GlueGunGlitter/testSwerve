// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class GramSubsystem extends SubsystemBase {
  /** Creates a new GramSubsystem. */
  //grap
  WPI_TalonFX motorM20 = new WPI_TalonFX(16);
  //ARM
  WPI_TalonFX motorM21 = new WPI_TalonFX(17);
  // bool
  boolean graund;
  public GramSubsystem() {
    motorM21.configForwardSoftLimitThreshold(0, 30);
    motorM21.configForwardSoftLimitEnable(true, 0);  
    motorM21.configReverseSoftLimitThreshold(0, 30);
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
    motorM21.configMotionCruiseVelocity(0);
    motorM21.configMotionAcceleration(0);

    //bool
    graund = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
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
  
  //Stop motorM20
  public void StopGrapper() {
  motorM20.set(0);
  }

  //set speed
  public void set(double speed) {
  motorM20.set(speed);
  }

  //get bool
  public boolean getbool() {
    return graund;
  }

  public void chengbool() {
    graund = !graund;
  }
}
