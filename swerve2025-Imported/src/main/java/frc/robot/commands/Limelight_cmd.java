
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.robotVision_SS;
//import edu.wpi.first.networktables.NetworkTable;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Limelight_cmd extends Command {
  /** Creates a new Limelight_cmd. */
  robotVision_SS vision;

  public Limelight_cmd(robotVision_SS subsystem) {
    this.vision = subsystem;
    addRequirements(subsystem);
  }

  public CoreCANrange range = new CoreCANrange(20, "SwerveBus");
  double rawDistance = range.getDistance().getValueAsDouble();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  HttpCamera limelightCam = new HttpCamera("Limelight", "http://10.75.31.11:5800/stream.mjpg");

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rawDistance = range.getDistance().getValueAsDouble();

    SmartDashboard.putNumber("Distance", rawDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}