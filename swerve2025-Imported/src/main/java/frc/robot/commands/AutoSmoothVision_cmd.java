// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.robotVision_SS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSmoothVision_cmd extends Command {
  /** Creates a new SmoothVision_cmd. */
  CommandSwerveDrivetrain drivetrain;
  robotVision_SS vision;
  CANrange range = new CANrange(20, "SwerveBus");

  public AutoSmoothVision_cmd(CommandSwerveDrivetrain subsystem, robotVision_SS visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = subsystem;
    this.vision = visionSubsystem;
    addRequirements(subsystem, visionSubsystem);
  }

  public SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  Transform2d offsetPose;

  PIDController rController = new PIDController(0.04, 0.03, 0);
  PIDController xController = new PIDController(1.7, 0, 0);
  PIDController yController = new PIDController(1.7, 0, 0);
  PIDController distanceController = new PIDController(2.8, 0, 0);

  enum State {
    DRIVE, ADVANCE, DONE
  }

  State state;

  int count;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.DRIVE;

    rController.reset();
    xController.reset();
    yController.reset();
    distanceController.reset();

    rController.setSetpoint(0);
    xController.setSetpoint(0);
    yController.setSetpoint(-0.4);
    distanceController.setSetpoint(0.2);

    rController.setTolerance(1);
    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    distanceController.setTolerance(0.05);

    rController.setIZone(5);

    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while ((state != State.DONE) && (count < 10000)) {
      NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      limelightTable.getEntry("pipeline").setNumber(0);
      double[] botPose = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[0]);
      long tid = limelightTable.getEntry("tid").getInteger(-1);
      double tx;
      double tz;
      double ry;
      SmartDashboard.putString("Pose", drivetrain.getState().Pose.toString());
      if (tid == -1) {
        if (offsetPose == null) {
          return;
        }
        Pose2d robotPose = drivetrain.getState().Pose.transformBy(offsetPose);
        tx = robotPose.getX();
        tz = robotPose.getY();
        ry = robotPose.getRotation().getDegrees();
      } else {
        tx = botPose[0];
        tz = botPose[2];
        ry = botPose[4];
        offsetPose = new Pose2d(tx, tz, Rotation2d.fromDegrees(ry)).minus(drivetrain.getState().Pose);
      }
      SmartDashboard.putNumber("TX", tx);
      SmartDashboard.putNumber("TZ", tz);
      SmartDashboard.putNumber("RY", ry);
      SmartDashboard.putBoolean("rControl", rController.atSetpoint());
      SmartDashboard.putBoolean("xControl", xController.atSetpoint());
      SmartDashboard.putBoolean("yControl", yController.atSetpoint());
      Double distance = range.getDistance().getValueAsDouble();
      double velDrive = 0;
      double velRotate = 0;
      double velStrafe = 0;
      switch (state) {
        case DRIVE:
          if (ry > 0) {
            velRotate = Math.min(rController.calculate(ry), 0.8);
          } else {
            velRotate = Math.max(rController.calculate(ry), -0.8);
          }
          velStrafe = -xController.calculate(tx);
          velDrive = yController.calculate(tz);
          if (rController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint()) {
            state = State.ADVANCE;
            System.out.println("Advance");
          }
          break;
        case ADVANCE:
          velDrive = -distanceController.calculate(distance);
          if (distanceController.atSetpoint()) {
            state = State.DONE;
            System.out.println("Done");
          }
          break;
        default:
          break;
      }
      driveRequest.withVelocityX(velDrive).withVelocityY(velStrafe).withRotationalRate(velRotate);
      drivetrain.setControl(driveRequest);
      count++;
    }
    if (count > 10000) {
      state = State.DONE;
      System.out.println("Auto command timed out!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }
}
