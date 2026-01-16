// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOuttake_cmd extends Command {
  private  Algae tempAlgae;
  private static final double AlgaeSpeed = -0.4;

  public AlgaeOuttake_cmd(Algae ss_Algae) {
    tempAlgae = ss_Algae;
    addRequirements(ss_Algae);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tempAlgae.AlgaeOuttake(AlgaeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tempAlgae.isDoneAlgae() == true) {
      
      //isDone turns off the motors and resets timer
      
      tempAlgae.ResetTimeComplete();
      return true;
    } else {
      return false;
    } 
   
  }
}
