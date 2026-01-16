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
public class SmoothVision_cmd extends Command {
  /** Creates a new SmoothVision_cmd. */
  CommandSwerveDrivetrain drivetrain;
  robotVision_SS vision;
  CANrange range = new CANrange(20, "SwerveBus");

  double rControlP = SmartDashboard.getNumber("rControlP", 0); //0.1
  double rControlI = SmartDashboard.getNumber("rControlI", 0); //0.03
  double rControlD = SmartDashboard.getNumber("rControlD", 0); //0.0

  double xControlP = SmartDashboard.getNumber("xControlP", 0); //1.7
  double xControlI = SmartDashboard.getNumber("xControlI", 0); //0.0
  double xControlD = SmartDashboard.getNumber("xControlD", 0); //0.0

  double yControlP = SmartDashboard.getNumber("yControlP", 0); //1.7
  double yControlI = SmartDashboard.getNumber("yControlI", 0); //0.0
  double yControlD = SmartDashboard.getNumber("yControlD", 0); //0.0

  double dControlP = SmartDashboard.getNumber("dControlP", 0); //1.7
  double dControlI = SmartDashboard.getNumber("dControlI", 0); //0.0
  double dControlD = SmartDashboard.getNumber("dControlD", 0); //0.0


  public SmoothVision_cmd(CommandSwerveDrivetrain subsystem, robotVision_SS visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = subsystem;
    this.vision = visionSubsystem;
    addRequirements(subsystem, visionSubsystem);
    
  }

  public SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  Pose2d targetPose;

  PIDController rController = new PIDController(rControlP, rControlI, rControlD);
  PIDController xController = new PIDController(xControlP, xControlI, xControlD);
  PIDController yController = new PIDController(yControlP, yControlI, yControlD);
  PIDController distanceController = new PIDController(dControlP, dControlI, dControlP);

  enum State {
    DRIVE, ADVANCE, DONE
  }

  State state;

  double lastTX = 0;
  double lastTY = 0;
  double lastTR = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.DRIVE;

    rController.reset();
    xController.reset();
    yController.reset();
    distanceController.reset();

    rControlP = SmartDashboard.getNumber("rControlP", 0); // 0.1
    rControlI = SmartDashboard.getNumber("rControlI", 0); // 0.03
    rControlD = SmartDashboard.getNumber("rControlD", 0); // 0.0

    xControlP = SmartDashboard.getNumber("xControlP", 0); // 1.7
    xControlI = SmartDashboard.getNumber("xControlI", 0); // 0.0
    xControlD = SmartDashboard.getNumber("xControlD", 0); // 0.0

    yControlP = SmartDashboard.getNumber("yControlP", 0); // 1.7
    yControlI = SmartDashboard.getNumber("yControlI", 0); // 0.0
    yControlD = SmartDashboard.getNumber("yControlD", 0); // 0.0

    rController.setP(rControlP);
    rController.setI(rControlI);
    rController.setD(rControlD);

    xController.setP(xControlP);
    xController.setI(xControlI);
    xController.setD(xControlD);

    yController.setP(yControlP);
    yController.setI(yControlI);
    yController.setD(yControlD);

    distanceController.setP(dControlP);
    distanceController.setI(dControlI);
    distanceController.setD(dControlD);

    rController.setSetpoint(0);
    xController.setSetpoint(0);
    yController.setSetpoint(-0.2);
    distanceController.setSetpoint(0.04);

    rController.setTolerance(0.5);
    xController.setTolerance(0.02);
    yController.setTolerance(0.1);
    distanceController.setTolerance(0.05);

    rController.setIZone(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    limelightTable.getEntry("pipeline").setNumber(0);
    double[] botPose = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[0]);
    long tid = limelightTable.getEntry("tid").getInteger(-1);
    double tx;
    double tz;
    double ry;
    
    if (tid == -1) {
      if (targetPose == null) {
        return;
      }
      Transform2d offsetGuess = drivetrain.getState().Pose.minus(targetPose);
      tx = -offsetGuess.getX();
      tz = offsetGuess.getY();
      ry = offsetGuess.getRotation().getDegrees();
    } else {
      // double temp;
      // SmartDashboard.putString("Pose", drivetrain.getState().Pose.toString());
      // temp = SmartDashboard.getNumber("rControlP", 0);
      // System.out.println(temp);
      tx = -botPose[0];
      tz = botPose[2];
      ry = botPose[4];

      lastTX = tx;
      lastTY = tz;
      lastTR = ry;
      Transform2d offTransform = new Transform2d(-tx, -tz, Rotation2d.fromDegrees(-ry));
      targetPose = drivetrain.getState().Pose.plus(offTransform);
    }
    Double distance = range.getDistance().getValueAsDouble();
    
    SmartDashboard.putNumber("TX", tx);
    SmartDashboard.putNumber("TZ", tz);
    SmartDashboard.putNumber("RY", ry);
    SmartDashboard.putNumber("DISTANCE", distance);

    SmartDashboard.putBoolean("rControl", rController.atSetpoint());
    SmartDashboard.putBoolean("xControl", xController.atSetpoint());
    SmartDashboard.putBoolean("yControl", yController.atSetpoint());
    SmartDashboard.putBoolean("dControl", distanceController.atSetpoint());

    SmartDashboard.putNumber("Last Tag X", lastTX);
    SmartDashboard.putNumber("Last Tag Y", lastTY);
    SmartDashboard.putNumber("Last Tag R", lastTR);

    double velDrive = 0;
    double velRotate = 0;
    double velStrafe = 0;
    switch (state) {
      case DRIVE:
        // if (ry > 0) {
        //   velRotate = Math.min(rController.calculate(ry), 0.8);
        // } else {
        //   velRotate = Math.max(rController.calculate(ry), -0.8);
        // }
        velDrive = yController.calculate(tz);
        if (!rController.atSetpoint() || !xController.atSetpoint()) {
          velRotate = rController.calculate(ry);
          velStrafe = xController.calculate(tx);
        }
        if (rController.atSetpoint() && xController.atSetpoint()) {
          if ((distance < Math.abs(yController.getSetpoint()) + yController.getErrorTolerance())&&range.getIsDetected().getValue()) {
            state = State.DONE;
            System.out.println("Done");
          }
        }
        break;
      default:
        break;
    }
    driveRequest.withVelocityX(velDrive).withVelocityY(velStrafe).withRotationalRate(velRotate);
    drivetrain.setControl(driveRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == State.DONE) {
      driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
      drivetrain.setControl(driveRequest);
      return true;
    } else {
      return false;
    }
  }
}
