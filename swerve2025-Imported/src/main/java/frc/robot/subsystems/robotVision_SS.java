// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class robotVision_SS extends SubsystemBase {
  /** Creates a new robotVision_SS. */
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  double tx; // Tag X on the camera
  double ty; // Tag Y on the camera

  double id;

  double[] targetPose;

  double skew;

  public robotVision_SS() {
    limelightTable.getEntry("camMode").setNumber(4);
  }

  public void setVariables() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    if (limelightTable != null) {
      tx = limelightTable.getEntry("tx").getDouble(0.0); // Tag X on the camera
      ty = limelightTable.getEntry("ty").getDouble(0.0); // Tag Y on the camera
      id = limelightTable.getEntry("tid").getDouble(-1);

      targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);
      skew = targetPose[4];
      
      SmartDashboard.putNumber("Skew", skew);
      SmartDashboard.putNumber("AdjustTy", ty + 1.08);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    if (limelightTable != null) {
      tx = limelightTable.getEntry("tx").getDouble(0.0); // Tag X on the camera
      ty = limelightTable.getEntry("ty").getDouble(0.0); // Tag Y on the camera
      id = limelightTable.getEntry("tid").getDouble(-1);

      targetPose = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);

      if (targetPose.length > 4) {
        skew = targetPose[4];
      }

      SmartDashboard.putNumber("Skew", skew);
      SmartDashboard.putNumber("Tag X", tx);
      SmartDashboard.putNumber("Tag Y", ty);
      SmartDashboard.putNumber("Tag ID", id);
      SmartDashboard.putString("LimeLight", "http://limelight.local:5800");
    }
  }

  public NetworkTable getTable() {
    return limelightTable;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public double[] getPose() {
    return targetPose;
  }

  public double getSkew() {
    return skew;
  }

  public double getId() {
    return id;
  }
}
