// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Grappersubsystem;


public class GrabCommand extends CommandBase {
  ColorMatch m_colorMatcher = new ColorMatch();
  Color kBlankTarget = new Color(87, 130, 79);
  Color kPurpleTarget = new Color(128,0,128);
  Color kYellowTarget = new Color(255,255,0);

  String colorString;
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  Color detectedColor = m_colorSensor.getColor();
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  /** Creates a new GrabCommand. */
  private Grappersubsystem m_Grapper;
  public GrabCommand(Grappersubsystem grapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Grapper = grapper;
    addRequirements(m_Grapper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_Grapper.CloseGrap(4);
      
    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Proximity",m_Grapper.getProximity());
    if( m_Grapper.getProximity() > 1800 )
    {
      m_Grapper.StopGrapper();
    }


  }



  
  // Called once the command ends or is interrupted.
  @Override
   public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
