// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShiftRight_cmd extends Command {
  /** Creates a new Shift_cmd. */
  public CommandSwerveDrivetrain drivetrain;
  public SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
  public double currentPosition;
  public double initialPosition;
  public boolean done = false;
  public ShiftRight_cmd(CommandSwerveDrivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = drivetrain.getState().ModulePositions[0].distanceMeters;
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = drivetrain.getState().ModulePositions[0].distanceMeters;
    if (Math.abs(initialPosition - currentPosition) < 0.17) {
      driveRequest.withVelocityY(-0.8);
      drivetrain.setControl(driveRequest);
    } else {
      driveRequest.withVelocityY(0);
      drivetrain.setControl(driveRequest);
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    System.out.println("Isfinished"+done);

    return done;
  }
}