// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeIntake_cmd;
import frc.robot.commands.AlgaeOuttake_cmd;
import frc.robot.commands.AlgaePivotDown_cmd;
import frc.robot.commands.AlgaePivotUp_cmd;
import frc.robot.commands.AlgaeAutoPivotUp_cmd;
import frc.robot.commands.AlgaeIdle_cmd;
import frc.robot.commands.AlgaeAutoPivotDown_cmd;
import frc.robot.commands.AlgaeAutoPivotFloor_cmd;
import frc.robot.commands.AlignRobotTag_cmd;
import frc.robot.commands.AutoSmoothVision_cmd;
import frc.robot.commands.CoralIntakeAuto_cmd;
import frc.robot.commands.CoralIntake_cmd;
import frc.robot.commands.CoralOutTakeAuto_cmd;
import frc.robot.commands.CoralOutTake_cmd;
import frc.robot.commands.ElevatorDOWNAuto_cmd;
import frc.robot.commands.ElevatorDOWN_cmd;
import frc.robot.commands.ElevatorLevel1_cmd;
import frc.robot.commands.ElevatorLevel2_cmd;
import frc.robot.commands.ElevatorLevel3_cmd;
import frc.robot.commands.ElevatorLevel4_cmd;
import frc.robot.commands.ElevatorBarge_cmd;
import frc.robot.commands.ElevatorAlgae1_cmd;
import frc.robot.commands.ElevatorAlgae2_cmd;
import frc.robot.commands.ElevatorUP_cmd;
import frc.robot.commands.HangDown_cmd;
import frc.robot.commands.Limelight_cmd;
import frc.robot.commands.ResetEncoder_cmd;
import frc.robot.commands.SmoothVision_cmd;
import frc.robot.commands.ShiftLeft_cmd;
import frc.robot.commands.ShiftRight_cmd;
import frc.robot.commands.StopDrivetrain_cmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral_SS;
import frc.robot.subsystems.robotVision_SS;
import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.NamedCommands;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    private final robotVision_SS m_robotVision_SS = new robotVision_SS();
    private final Coral_SS ss_coral = new Coral_SS();
    private final Algae algae = new Algae();
    private double lastYAxisValue = 0.0;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem Elevator = new ElevatorSubsystem();

    //Establishing pathplanner Named Commands
    AlignRobotTag_cmd align = new AlignRobotTag_cmd(drivetrain, m_robotVision_SS);
    AlgaeAutoPivotDown_cmd algaeAutoPivotDown = new AlgaeAutoPivotDown_cmd(algae);
    AlgaeAutoPivotFloor_cmd algaeAutoPivotFloor = new AlgaeAutoPivotFloor_cmd(algae);
    AlgaeAutoPivotUp_cmd algaeAutoPivotUp = new AlgaeAutoPivotUp_cmd(algae);
    AlgaeAutoPivotFloor_cmd algaeAutoPivotUP = new AlgaeAutoPivotFloor_cmd(algae);
    ShiftLeft_cmd shift_left = new ShiftLeft_cmd(drivetrain);
    ShiftRight_cmd shift_right = new ShiftRight_cmd(drivetrain);
    AutoSmoothVision_cmd autoAl = new AutoSmoothVision_cmd(drivetrain, m_robotVision_SS);
    SmoothVision_cmd smoothAl = new SmoothVision_cmd(drivetrain, m_robotVision_SS);
    Limelight_cmd runVision = new Limelight_cmd(m_robotVision_SS);
    CoralIntakeAuto_cmd coralIntake = new CoralIntakeAuto_cmd(ss_coral);
    CoralOutTakeAuto_cmd coralOuttake = new CoralOutTakeAuto_cmd(ss_coral);
    ElevatorLevel2_cmd elevatorLevel2 = new ElevatorLevel2_cmd(Elevator);
    ElevatorLevel3_cmd elevatorLevel3 = new ElevatorLevel3_cmd(Elevator);
    ElevatorDOWNAuto_cmd elevatorDOWNAuto = new ElevatorDOWNAuto_cmd(Elevator);
    ElevatorLevel4_cmd elevatorLevel4 = new ElevatorLevel4_cmd(Elevator);
    ElevatorBarge_cmd elevatorBarge = new ElevatorBarge_cmd(Elevator);
    ElevatorLevel1_cmd elevatorLevel1 = new ElevatorLevel1_cmd(Elevator);
    double speedFactor = 0.3;
    AlgaeOuttake_cmd algaeOuttake = new AlgaeOuttake_cmd(algae);
    AlgaeIntake_cmd algaeIntake = new AlgaeIntake_cmd(algae);
    AlgaeIdle_cmd algaeIdle = new AlgaeIdle_cmd(algae);

    ElevatorAlgae1_cmd elevatorAlgae1 = new ElevatorAlgae1_cmd(Elevator);
    ElevatorAlgae2_cmd elevatorAlgae2 = new ElevatorAlgae2_cmd(Elevator);

    public RobotContainer() 
    {
        //Establishing pathplanner Named Commands
        NamedCommands.registerCommand("AlignRobotTag_cmd", align);
        NamedCommands.registerCommand("AutoAlignRobot_cmd", smoothAl);
        NamedCommands.registerCommand("AlgaeAutoPivotDown_cmd", algaeAutoPivotDown);
        NamedCommands.registerCommand("AlgaeAutoPivotFloor_cmd", algaeAutoPivotFloor);
        NamedCommands.registerCommand("AlgaeAutoPivotUp_cmd", algaeAutoPivotUp);
        NamedCommands.registerCommand("AlgaeOuttake_cmd", algaeOuttake);
        NamedCommands.registerCommand("AlgaeIntakeAuto_cmd", algaeIntake);
        NamedCommands.registerCommand("CoralIntakeAuto_cmd", coralIntake);
        NamedCommands.registerCommand("CoralOuttakeAuto_cmd", coralOuttake);
        NamedCommands.registerCommand("ElevatorLevel2_cmd", elevatorLevel2);
        NamedCommands.registerCommand("ElevatorLevel3_cmd", elevatorLevel3);
        NamedCommands.registerCommand("ElevatorDOWNAuto_cmd", elevatorDOWNAuto);
        NamedCommands.registerCommand("ElevatorLevel4_cmd", elevatorLevel4);
        NamedCommands.registerCommand("ElevatorAlgae1_cmd", elevatorAlgae1);
        NamedCommands.registerCommand("ElevatorAlgae2_cmd", elevatorAlgae2);
        NamedCommands.registerCommand("ElevatorLevel1_cmd", elevatorLevel1);
        NamedCommands.registerCommand("ShiftRight_cmd", shift_right);
        NamedCommands.registerCommand("ShiftLeft_cmd", shift_left);
        NamedCommands.registerCommand("ElevatorBarge_cmd", elevatorBarge);
        NamedCommands.registerCommand("AlgaeAutoPiviotFloor_cmd", algaeAutoPivotFloor);
        NamedCommands.registerCommand("AlgaeIdle_cmd", algaeIdle);
        

        autoChooser = AutoBuilder.buildAutoChooser("None");

        SmartDashboard.putData("Auto Mode", autoChooser);

       /*autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
              ? stream.filter(auto -> auto.getName().startsWith("Red"))
              : stream
          );*/

        
        configureBindings();        
        //algae.status();
        
        m_robotVision_SS.setDefaultCommand(runVision);
        double tx = m_robotVision_SS.getTx();      

      
    }
      private double getSpeed(){

        
       if (Elevator.getEncoderPosition() > 13) {
           speedFactor = 0.15;
       } else if (Elevator.getEncoderPosition() > 2) {
           speedFactor = 0.3;
       } else {
           speedFactor= 0.65;  //was .6
       }

        return speedFactor;
      }
    private void configureBindings() 
    {
        CameraServer.startAutomaticCapture();
       
        drivetrain.setDefaultCommand(

            
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(getSpeed() * -controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(getSpeed()* -controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(getSpeed() * -controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left) 
                )
            );
            /*drivetrain.applyRequest(() -> 
                drive.withVelocityX(0.4 * -controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(0.4 * -controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(0.4 * -controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left) 
                )
            );*/
        
      //controller2//////////////////////////////////////////////////////////////////////////////////////////////////////

        controller2.rightTrigger().whileTrue(new ElevatorUP_cmd(Elevator));
        controller2.leftTrigger().whileTrue(new ElevatorDOWN_cmd(Elevator));
        //These triggers will be used for algae once code is tested/merged
      /*  new Trigger(() -> joystick2.getLeftY() < .1)
        .onTrue(new ElevatorUP_cmd(Elevator));
        new Trigger(() -> joystick2.getLeftY() < -.1)
        .onTrue(new ElevatorDOWN_cmd(Elevator));
*/
        /*new Trigger(() -> {
            double currentPosition = joystick2.getLeftY();
            boolean hasChanged = Math.abs(currentPosition - lastYAxisValue) > .04;
            if (hasChanged) {
                lastYAxisValue = currentPosition;
            }
            return hasChanged;
        }).onTrue(new ElevatorUP_cmd(Elevator));
        */

        controller2.povLeft().onTrue(new  ElevatorAlgae1_cmd(Elevator));
        controller2.povRight().onTrue(new  ElevatorAlgae2_cmd(Elevator));
        controller2.povUp().onTrue(new  ElevatorBarge_cmd(Elevator));
        controller2.povDown().onTrue(new  ElevatorDOWNAuto_cmd(Elevator));
        controller2.start().onTrue(new HangDown_cmd(Elevator));

        controller2.a().onTrue(new ElevatorLevel1_cmd(Elevator));
        controller2.b().onTrue(new ElevatorLevel2_cmd(Elevator));
        controller2.x().onTrue(new ElevatorLevel3_cmd(Elevator));
        controller2.y().onTrue(new ElevatorLevel4_cmd(Elevator));
        controller2.leftStick().whileTrue(new ResetEncoder_cmd(Elevator));
        
        controller2.leftBumper().whileTrue(new CoralOutTake_cmd(ss_coral));
        controller2.rightBumper().whileTrue(new CoralIntake_cmd(ss_coral));
        


    ///Controller1/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        controller.rightBumper().onTrue(algaeIntake);
        controller.leftBumper().onTrue(algaeOuttake);
        controller.leftTrigger().whileTrue(new AlgaePivotUp_cmd(algae));
        controller.rightTrigger().whileTrue(new AlgaePivotDown_cmd(algae));
        controller.povUp().onTrue(algaeAutoPivotUp);
        controller.povDown().onTrue(algaeAutoPivotDown);
        
        controller.povRight().onTrue(algaeIdle);
        controller.povLeft().onTrue(algaeAutoPivotFloor);
        //controller.leftTrigger().onFalse(new AlgaeIdle_cmd(algae));
        //controller.rightTrigger().onFalse(new AlgaeIdle_cmd(algae));    

        controller.start().and(controller.y()).whileTrue(new ElevatorUP_cmd(Elevator));
        controller2.back().and(controller2.y()).whileTrue(new ElevatorDOWN_cmd(Elevator));

        controller.y().whileTrue(drivetrain.pigeonCommand());
        /*controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));*/
        controller.x().whileTrue(smoothAl);
        controller.start().whileTrue(align);
        controller.a().onTrue(shift_left);
        controller.b().onTrue(shift_right);

        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() 
    {
        System.out.println("Selected Auto: " + autoChooser.getSelected());
        return autoChooser.getSelected();
    }
}
