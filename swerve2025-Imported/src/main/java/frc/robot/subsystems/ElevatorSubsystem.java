// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX kleftElevatorMotor = new TalonFX (42);
  private final TalonFX kRightElevatorMotor= new TalonFX(44);
  private static final double ElevatorSpeed = 0.35;
  private static final double DownElevatorSpeed = -0.1;
  private static boolean positionAchieved = false;
  private static double ElevatorStopSpeed = 0.05;
  private int counter = 0;
  private static double ElevatorTolerance = 1; 


  private final TalonFXSimState simLeftElevator = new TalonFXSimState(kleftElevatorMotor);
  private final TalonFXSimState simRightElevator = new TalonFXSimState(kRightElevatorMotor);
  double elevatorEncoderPosition = 0;
  public void simulationPeriodic() {
    simLeftElevator.setRawRotorPosition(elevatorEncoderPosition);
    simRightElevator.setRawRotorPosition(elevatorEncoderPosition);
  }

  public ElevatorSubsystem() {
    setDefaultCommand(new RunCommand(() -> StopMotors(), this));
    setEncoderPositionInit(); 
    counter = 0;
  }

  public void StopMotors() {
    kleftElevatorMotor.set(0); //0.036
    kRightElevatorMotor.set(0); //0.036

  }

  public void moveToPositionUP(double desiredPosition) {
    double currentrobotpostition = kleftElevatorMotor.getPosition().getValueAsDouble();
    if(currentrobotpostition < desiredPosition) {
      setElevatorSpeed(ElevatorSpeed);
      elevatorEncoderPosition = elevatorEncoderPosition + 0.5;
    } else if (currentrobotpostition > desiredPosition + ElevatorTolerance) {
      setElevatorSpeed(DownElevatorSpeed);
      elevatorEncoderPosition = elevatorEncoderPosition - 0.5;
    } else {
      StopMotors();
      positionAchieved = true;
      counter++;
    }
  }

  public void moveToPositionDOWN(double desiredPosition) {
    double currentrobotpostition = kleftElevatorMotor.getPosition().getValueAsDouble();
    if(currentrobotpostition >= desiredPosition) {
      //setElevatorSpeed(ElevatorSpeed);
      System.out.printf("Elevator desiredPOSITION", desiredPosition, "> currentrobotposition", currentrobotpostition);
    } else {
      StopMotors();
      positionAchieved = true;
      counter++;
    }
  }

  public double getEncoderPosition() {
  return kleftElevatorMotor.getPosition().getValueAsDouble();
  }

  public void setEncoderPositionInit() {
    kleftElevatorMotor.setPosition(0);
    kRightElevatorMotor.setPosition(0);
  }
     
  public boolean getAchievement() {
    return positionAchieved;
  }

  public void setAchievement(boolean achievement) {
    positionAchieved = achievement;    
  }

  public void setElevatorSpeed(double speed) {
    setSpeed(speed, kleftElevatorMotor);
    setSpeed(speed, kRightElevatorMotor);
  }

  public void setElevatorHangSpeed(double speed) {
    kleftElevatorMotor.set(speed);
    kRightElevatorMotor.set(speed);
  }

  public void setSpeed(double speed, TalonFX motor) {
    double tune_Elevator_DownSlowSpeed = 0.08;
    double tune_Elevator_UpSlowSpeed = .28;
    double tune_Elevator_EncoderPositionSlow = 10;
    if (getEncoderPosition() < tune_Elevator_EncoderPositionSlow) {
      if (speed > 0) {
        motor.set (tune_Elevator_UpSlowSpeed);
      } else {
        motor.set(tune_Elevator_DownSlowSpeed * -1);
        System.out.println("DOWN SLOW SPEED ");
      }
    } else {
      motor.set(speed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder", kRightElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Encoder", kleftElevatorMotor.getPosition().getValueAsDouble());
    //Command currentCommand = CommandScheduler.getInstance().requiring(this);
  }
}