// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.robotVision_SS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobotTag_cmd extends Command {
  /** Creates a new AlignRobotTag_cmd. */
  public CommandSwerveDrivetrain drivetrain;
  public SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
  robotVision_SS vision = new robotVision_SS();

  public CoreCANrange range = new CoreCANrange(20, "SwerveBus");
  double distance = range.getDistance().getValueAsDouble();

  public AlignRobotTag_cmd(CommandSwerveDrivetrain subsystem, robotVision_SS visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = visionSubsystem;
    this.drivetrain = subsystem;
    addRequirements(subsystem);
  }

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  double tx = limelightTable.getEntry("tx").getDouble(0.0); // Tag X on the camera
  double ty = limelightTable.getEntry("ty").getDouble(0.0); // Tag Y on the camera
  double id = limelightTable.getEntry("tid").getDouble(-1);

  double[] targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);

  double skew = 0;

  //Skew Tolerances at the checkpoints/reef
  double skewToleranceAtPoint3 = 5;
  double skewToleranceAtPoint2 = 2.5;
  double skewToleranceAtPoint1 = 1;
  double skewFinalTolerance = 0.25;

  //Tx Tolerance at the checkpoints/reef
  double txToleranceAtPoint3 = 7.5;
  double txToleranceAtPoint2 = 4.5;
  double txToleranceAtPoint1 = 1.5;
  double txFinalTolerance = 0.25;

  //Distance at the checkpoints/reef
  double point3Distance = 0.9;
  double point2Distance = 0.67;
  double point1Distance = 0.6;
  double finalDistance = 0.2;

  boolean skewDone = false;
  boolean txDone = false;
  boolean distanceDone = false;

  boolean reachedPoint3 = false;
  boolean reachedPoint2 = false;
  boolean reachedPoint1 = false;
  boolean reachedFinal = false;

  double txVelocity = 0;
  double skewVelocity = 0;
  double distanceVelocity = 0;

  //Speed to align Tx while going to checkpoints/reef
  //Left/Right
  double txSpeedMultiplierAtPoint3 = 0.9;
  double txSpeedMultiplierAtPoint2 = 0.65;
  double txSpeedMultiplierAtPoint1 = 0.35;
  double txFinalSpeedMultiplier = 0.2;

  //Speed to align Skew while going to checkpoints/reef
  //Rotation
  double skewSpeedMultiplierAtPoint3 = 1;
  double skewSpeedMultiplierAtPoint2 = 0.85;
  double skewSpeedMultiplierAtPoint1 = 0.75;
  double skewFinalSpeedMultiplier = 0.5;

  //Speed to align Distance while going to checkpoints/reef
  //Forward
  double distanceSpeedMultiplierAtPoint3 = 1.1;
  double distanceSpeedMultiplierAtPoint2 = 0.85;
  double distanceSpeedMultiplierAtPoint1 = 0.6;
  double distanceFinalSpeedMultiplier = 0.75;

  boolean idSet = false;

  boolean isdone;

  double lastTX = 0;
  double lastSkew = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<throttle_set>").setNumber(0);
    isdone = false;
    skewDone = false;
    txDone = false;
    distanceDone = false;

    reachedPoint3 = false;
    reachedPoint2 = false;
    reachedPoint1 = false;
    reachedFinal = false;

    id = limelightTable.getEntry("tid").getDouble(-1);
    distance = range.getDistance().getValueAsDouble();

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);

    if (targetPose.length != 0) {
      tx = limelightTable.getEntry("tx").getDouble(0.0);
      ty = limelightTable.getEntry("ty").getDouble(0.0);

      skew = targetPose[4]; // Safe to access index 4
    } else {
      skew = 0; // Safer to access index 4
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);
    if (targetPose.length != 0) {
      if (idSet == false) {
        if (limelightTable.getEntry("tid").getDouble(-1) != -1) {
          id = limelightTable.getEntry("tid").getDouble(-1);
          idSet = true;
        }
      }

      txVelocity = 0;
      skewVelocity = 0;
      distanceVelocity = 0;

      distance = range.getDistance().getValueAsDouble();
      limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

      if ((id == limelightTable.getEntry("tid").getDouble(-1))) {
        tx = limelightTable.getEntry("tx").getDouble(0.0);
        ty = limelightTable.getEntry("ty").getDouble(0.0);

        targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);
        skew = targetPose[4];
      }

      if (tx != 0) {
        lastTX = tx;
      }
      if (skew != 0) {
        lastSkew = skew;
      }

      if (reachedPoint3 == false) {
        //If we are not at checkpoint 3
  
        if (skewDone == false) {
          // Align Skew
          if ((Math.abs(skew) > skewToleranceAtPoint3)&&(skew != 0)) {
            skewVelocity = -skewSpeedMultiplierAtPoint3 * Math.signum(skew);
          } else {
            skewDone = true;
          }
        } else if (txDone == false) {
          // Align Tx
          if ((Math.abs(tx) > txToleranceAtPoint3)&&(tx != 0)) {
            txVelocity = txSpeedMultiplierAtPoint3 * Math.signum(tx);
          } else {
            txDone = true;
          }
        } else if (distanceDone == false) {
          // Go to Distance Point 3
          if (distance > point3Distance) {
            distanceVelocity = distanceSpeedMultiplierAtPoint3;
          } else {
            distanceDone = true;
          }
        } else {
          reachedPoint3 = true;
          skewDone = false;
          txDone = false;
          distanceDone = false;
        }
      } else if (reachedPoint2 == false) {
        //If we are not at checkpoint 2
  
        if (skewDone == false) {
          // Align Skew
          if ((Math.abs(skew) > skewToleranceAtPoint2)&&(skew != 0)) {
            skewVelocity = -skewSpeedMultiplierAtPoint2 * Math.signum(skew);
          } else {
            skewDone = true;
          }
        } else if (txDone == false) {
          // Align Tx
          if ((Math.abs(tx) > txToleranceAtPoint2)&&(tx != 0)) {
            txVelocity = txSpeedMultiplierAtPoint2 * Math.signum(tx);
          } else {
            txDone = true;
          }
        } else if (distanceDone == false){
          // Go to Distance Point 2
          if (distance > point2Distance) {
            distanceVelocity = distanceSpeedMultiplierAtPoint2;
          } else {
            distanceDone = true;
          }
        } else {
          reachedPoint2 = true;
          skewDone = false;
          txDone = false;
          distanceDone = false;
        }
      } else if (reachedPoint1 == false) {
        //If we are not at checkpoint 1

        if (skewDone == false) {
          // Align Skew
          if ((Math.abs(skew) > skewToleranceAtPoint1)&&(skew != 0)) {
            skewVelocity = -skewSpeedMultiplierAtPoint1 * Math.signum(skew);
          } else {
            skewDone = true;
          }
        } else if (txDone == false) {
          // Align Tx
          if ((Math.abs(tx) > txToleranceAtPoint1)&&(tx != 0)) {
            txVelocity = txSpeedMultiplierAtPoint1 * Math.signum(tx);
          } else {
            txDone = true;
          }
        } else if (distanceDone == false){
          // Go to Distance Point 1
          if (distance > point1Distance) {
            distanceVelocity = distanceSpeedMultiplierAtPoint1;
          } else {
            distanceDone = true;
          }
        } else {
          reachedPoint1 = true;
          skewDone = false;
          txDone = false;
          distanceDone = false;
        }
      } else if (reachedFinal == false) {
        //If we are not at the reef

        if (skewDone == false) {
          // Align Skew
          if ((Math.abs(skew) > skewFinalTolerance)&&(skew != 0)) {
            skewVelocity = -skewFinalSpeedMultiplier * Math.signum(skew);
          } else {
            skewDone = true;
          }
        } else if (txDone == false) {
          // Align Tx
          if ((Math.abs(tx) > txFinalTolerance)&&(tx != 0)) {
            txVelocity = txFinalSpeedMultiplier * Math.signum(tx);
          } else {
            txDone = true;
          }
        } else if (distanceDone == false) {
          // Go to Distance Final
          if (distance > finalDistance) {
            distanceVelocity = distanceFinalSpeedMultiplier;
          } else {
            distanceDone = true;
          }
        } else {
          System.out.println("Completed! You should be aligned.");
          reachedFinal = true;
          txDone = false;
          skewDone = false;
          idSet = false;
          distanceDone = false;
        }
      }
    }
    SmartDashboard.putBoolean("Skew Done", skewDone);
    SmartDashboard.putBoolean("Distance Done", distanceDone);
    SmartDashboard.putBoolean("TX Done", txDone);
    SmartDashboard.putBoolean("ID Set", idSet);
    SmartDashboard.putNumber("Last TX", lastTX);
    SmartDashboard.putNumber("Last Skew", lastSkew);
    

    drivetrain.setControl(
        driveRequest.withVelocityX(distanceVelocity).withVelocityY(txVelocity).withRotationalRate(skewVelocity));

    if (skewDone && txDone && reachedFinal) {
      isdone = true;
    } else {
      isdone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<throttle_set>").setNumber(100);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isdone;
  }
}
