// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final TalonFX kleftElevatorMotor = new TalonFX (42);
    private final TalonFX kRightElevatorMotor= new TalonFX(44);
    

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final Pigeon2 pidgey = new Pigeon2(50, "SwerveBus");
    


  public Robot() {
    m_robotContainer = new RobotContainer();
    
   

  }

  
  @Override
  public void robotInit() {
   
    SmartDashboard.putNumber("rControlP", 0.135);
    SmartDashboard.putNumber("rControlI", 0.03);
    SmartDashboard.putNumber("rControlD", 0.00085);

    SmartDashboard.putNumber("xControlP", 2.0);
    SmartDashboard.putNumber("xControlI", 0.08);
    SmartDashboard.putNumber("xControlD", 0.0);
    
    SmartDashboard.putNumber("yControlP", 2.0);
    SmartDashboard.putNumber("yControlI", 0.04);
    SmartDashboard.putNumber("yControlD", 0.0);

    SmartDashboard.putNumber("dControlP", 0.0);
    SmartDashboard.putNumber("dControlI", 0.0);
    SmartDashboard.putNumber("dControlD", 0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<throttle_set>").setNumber(2);
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    
  

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<throttle_set>").setNumber(500);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //this.pidgey.setYaw(0);

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
