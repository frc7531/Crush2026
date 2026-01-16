// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUP_cmd extends Command {

  private final ElevatorSubsystem tempElevatorSubsystem;
  private static final double Elevator_UP_SPEED = 0.1;

  public ElevatorUP_cmd(ElevatorSubsystem ss_Elevator) 
  {
    tempElevatorSubsystem = ss_Elevator;
    addRequirements(ss_Elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    this.tempElevatorSubsystem.setElevatorSpeed(Elevator_UP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
