// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevel2_cmd extends Command {
 
  private final ElevatorSubsystem tempElevatorSubsystem;

  private final double tune_Elevator_Level2targetPos = 11.9;


  public ElevatorLevel2_cmd(ElevatorSubsystem ss_Elevator) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    tempElevatorSubsystem = ss_Elevator;  
    addRequirements(ss_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    tempElevatorSubsystem.moveToPositionUP(tune_Elevator_Level2targetPos);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

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
