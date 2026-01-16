// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAlgae2_cmd extends Command {

  private final ElevatorSubsystem tempElevatorSubsystem;
  private final double ElevatorAlgae2TargetPos = 14;
 
  public ElevatorAlgae2_cmd(ElevatorSubsystem ss_Elevator) 
  {
    tempElevatorSubsystem = ss_Elevator;
    addRequirements(ss_Elevator);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    tempElevatorSubsystem.moveToPositionUP(ElevatorAlgae2TargetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
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
