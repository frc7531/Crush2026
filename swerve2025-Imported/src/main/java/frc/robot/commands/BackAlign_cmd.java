// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BackAlign_cmd extends Command {
  /** Creates a new BackAlign_cmd. */
  public CommandSwerveDrivetrain drivetrain;
  public SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
  public CoreCANrange range = new CoreCANrange(32, "SwerveBus");

  public BackAlign_cmd(CommandSwerveDrivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = subsystem;
  }

  double distance = range.getDistance().getValueAsDouble();
  double distanceTolerance = 0.1;
  boolean done = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = range.getDistance().getValueAsDouble();
    if (distance > distanceTolerance) {
      drivetrain.setControl(
          driveRequest.withVelocityX(-0.2).withVelocityY(0).withRotationalRate(0));
    } else {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
