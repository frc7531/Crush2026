// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetEncoder_cmd extends Command 
{
  private final ElevatorSubsystem tempElevatorSubsystem;

  public ResetEncoder_cmd(ElevatorSubsystem ss_Elevator) 
  {
    tempElevatorSubsystem = ss_Elevator;
    addRequirements(ss_Elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    tempElevatorSubsystem.setEncoderPositionInit();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() 
  {
    return false;
  }
  }


