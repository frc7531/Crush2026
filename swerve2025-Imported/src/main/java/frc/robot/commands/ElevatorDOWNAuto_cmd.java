// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDOWNAuto_cmd extends Command 
{
  private final ElevatorSubsystem tempElevatorSubsystem;
  private final double tune_Elevator_DOWN_SPEED = -0.2;
  private final double tune_Elevator_DOWNTargetPos = 0.2;
  

  public ElevatorDOWNAuto_cmd(ElevatorSubsystem ss_Elevator) 
  {
    tempElevatorSubsystem = ss_Elevator;
    addRequirements(ss_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    this.tempElevatorSubsystem.setElevatorSpeed(tune_Elevator_DOWN_SPEED);
    tempElevatorSubsystem.moveToPositionDOWN(tune_Elevator_DOWNTargetPos);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (tempElevatorSubsystem.getAchievement()==true)
    {
      tempElevatorSubsystem.setAchievement(false);
      
      return true;
    } 
    else {

    return false;
    }
  }
}
