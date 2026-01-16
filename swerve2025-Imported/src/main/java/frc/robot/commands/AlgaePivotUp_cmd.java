// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivotUp_cmd extends Command {
  /** Creates a new AlgaePivotUp_cmd. */
  private  Algae tempAlgae;
  private static final double tune_Algae_UpPivotSpeed = .2;
  public AlgaePivotUp_cmd(Algae ss_Algae) {
    tempAlgae = ss_Algae;
    addRequirements(ss_Algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tempAlgae.setAlgaePivotSpeed(tune_Algae_UpPivotSpeed);
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
