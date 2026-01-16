// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAutoPivotFloor_cmd extends Command {
  private  Algae tempAlgae;
  private static final double tune_Algae_DownIntakeSpeed = 0.5;
  private static final double tune_Algae_DownEncoderPos = 9.5;

  public AlgaeAutoPivotFloor_cmd(Algae ss_Algae) {
    tempAlgae = ss_Algae;
    addRequirements(ss_Algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.tempAlgae.moveToPositionDOWN(tune_Algae_DownEncoderPos);
    this.tempAlgae.AlgaeOuttake(tune_Algae_DownIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tempAlgae.getAchievement()==true) {
      tempAlgae.setAlgaeSpeed(.03);
      tempAlgae.setAlgaePivotSpeed(-.02);
      tempAlgae.setAchievement(false);
      return true;
    } else {
      return false;
    }

  }
}

