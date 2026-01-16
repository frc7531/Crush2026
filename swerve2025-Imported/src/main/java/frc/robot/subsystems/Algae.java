// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Algae extends SubsystemBase {

  public void status() {}

  //Algae pivot motor
  private final SparkMax AlgaeIntakePivot = new SparkMax(40, MotorType.kBrushless);
  private final RelativeEncoder encoder = AlgaeIntakePivot.getEncoder();
  //Algae top motor
  private final SparkMax AlgaeIntakeTop = new SparkMax(51, MotorType.kBrushless);
  //Algae bottom motor
  private final SparkMax AlgaeIntakeBottom = new SparkMax(60, MotorType.kBrushless);

  private static boolean positionAlgaeAchieved = false;
  private static boolean AlgaeIdleAchieved = false;

  public Algae() {
   //setDefaultCommand(new RunCommand(() -> , this));
   encoder.setPosition(0);
  }

  public void setAlgaeSpeed(double speed) {
    AlgaeIntakeTop.set(speed);
    AlgaeIntakeBottom.set(speed);
  }

  public double getAlgaeSpeed(){
  
    return AlgaeIntakeTop.get();

  }


  public void setAlgaeSpeedAuto(double speed) {
    AlgaeIntakeTop.set(speed);
    AlgaeIntakeBottom.set(speed);
  }

  public void StopIntakeMotors() {
    AlgaeIntakeTop.set(0);
    AlgaeIntakeBottom.set(0);
  }

  public void AlgaeIdle() {
    AlgaeIntakeTop.set(0.1);
    AlgaeIntakeBottom.set(0.1);
  }

  public void setAlgaePivotSpeed(double speed) {
    AlgaeIntakePivot.set(speed);
    System.out.print(speed);
    AlgaetimerIdle++;
  }
  public void AlgaePivotStop() {
    AlgaeIntakePivot.set(0.05);
  }
  
  boolean timeComplete = false;
  int Algaetimer = 0;
  private final int tune_algae_timer = 800;
  
public boolean isDoneAlgae() {
  System.out.println(Algaetimer);
  if (Algaetimer >= tune_algae_timer/20) {
    this.StopIntakeMotors();
    Algaetimer = 0;
    timeComplete = true;

    return true;
  } 
  else {
    return false;
  }
}
public void ResetTimeComplete() {
  timeComplete = false;
}
public void ResetIdleTimeComplete() {
  timeCompleteIdle = false;
}
public void setIdleAchievement(boolean idleachievement) {
  AlgaeIdleAchieved = idleachievement;    
}
public boolean getIdleAchievement() {
  return AlgaeIdleAchieved;
}

boolean timeCompleteIdle = false;
int AlgaetimerIdle = 0;
private final int tune_algae_timer_idle = 10000;

public boolean isDoneAlgaeIdle() {
System.out.println(AlgaetimerIdle);
if (AlgaetimerIdle >= tune_algae_timer_idle/20) {
  AlgaetimerIdle = 0;
  timeCompleteIdle = true;
  AlgaeIdleAchieved = true;
  return true;
} 
else {
  return false;
}
}

  public void AlgaeOuttake(double speed) {
    AlgaeIntakeTop.set(speed);
    AlgaeIntakeBottom.set(speed);
    //Increase Counter by 1
    Algaetimer++;
  }

  public boolean getAchievement() {
    return positionAlgaeAchieved;
  }

  public void setAchievement(boolean achievement) {
    positionAlgaeAchieved = achievement;    
  }
 

  public void BottomAlgaeSpeedAuto(double motorspeed) {
    AlgaeIntakeBottom.set(motorspeed);
    //Increase Counter by 1
    Algaetimer++;
  }
  public void AlgaePivotDown_cmd(double speed) {
   AlgaeIntakePivot.set(speed);
  }
  public void AlgaePivotUp_cmd(double speed) {
    AlgaeIntakePivot.set(speed);
  }
  public void moveToPositionDOWN(double desiredPosition) {
    double currentrobotpostition = encoder.getPosition();

    System.out.println("algae" + desiredPosition);
    System.out.println("algae" + currentrobotpostition);
    if(currentrobotpostition <= desiredPosition) {
      setAlgaePivotSpeed(-0.3); //-.15
    } else {
      AlgaePivotStop();
      positionAlgaeAchieved = true;
    }
  }

  public void moveToPositionUP(double desiredPosition) {
    double currentrobotpostition = encoder.getPosition();
    if(currentrobotpostition > desiredPosition) {
      setAlgaePivotSpeed(0.5); //.25 
      } else {  
        AlgaePivotStop();
        positionAlgaeAchieved = true;
      }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Encoder Position", encoder.getPosition());
    SmartDashboard.putNumber("Algae TopMotorSpeed", this.AlgaeIntakeTop.get());

    if( this.AlgaeIntakePivot.get() == -.02) {
    } else {
      this.setAlgaePivotSpeed(-.01);
    }
  } 
}
