// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_SS;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutTake_cmd extends Command {
  /** Creates a new Coral_cmd. */
  private final Coral_SS ss_coral;
  final double tune_coral_topmotorOuttake = 0.25;
  final double tune_coral_bottommotorOuttake = -0.25;

  public CoralOutTake_cmd(Coral_SS temp_ss_Coral) 
  {
    ss_coral = temp_ss_Coral;
    addRequirements(temp_ss_Coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    this.ss_coral.TopSpeed(tune_coral_topmotorOuttake);
    this.ss_coral.BottomSpeed(tune_coral_bottommotorOuttake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  //if you run this command in auto, it will never "finish" so you want to work on that 
  public boolean isFinished() 
  {
   
      return false; 
  }
}
