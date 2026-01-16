// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral_SS extends SubsystemBase {
  /* Creates a new Coral. */

  //motors for outtake 
  private final SparkMax  m_topCoral = new SparkMax(62, MotorType.kBrushless);
  public final SparkMax  m_bottomCoral = new SparkMax(29, MotorType.kBrushless);
 
  
  //color sensor for coral funnel
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
   Color detectedColor = m_colorSensor.getColor();

    boolean timeComplete = false;
    int timer = 0;
    private final int tune_coral_timer = 1000;
    
public boolean isDone()
{
  if (timer >= tune_coral_timer/20) 
  {
    this.StopMotors();
    timer = 0;
    timeComplete = true;
    return true;
  }
  else 
  {
    return false;
  }
}

public void ResetTimeComplete(){
  timeComplete = false;
}

  public void TopSpeedAuto(double motorspeed)
  {
    m_topCoral.set(motorspeed);
    //Increase Counter by 1
    timer++;
  }

  public void BottomSpeed(double motorspeed){
    m_bottomCoral.set(motorspeed);
  }

  public void TopSpeed(double motorspeed)
  {
    m_topCoral.set(motorspeed);
    
  }



  public void StopMotors(){
   m_bottomCoral.set(0);
   m_topCoral.set(0);
  }
  public Coral_SS() {
    setDefaultCommand(new RunCommand(() -> StopMotors(), this));
    timeComplete = false;
    timer = 0;
  }
  
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
  
}
