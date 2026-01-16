/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
package frc.robot.commands;
import frc.robot.subsystems.Algae;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;


public class AlgaeIdle_cmd extends Command {
  private  Algae tempAlgae;


  public AlgaeIdle_cmd(Algae ss_Algae) {
    tempAlgae = ss_Algae;
    addRequirements(ss_Algae);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tempAlgae.setAlgaePivotSpeed(.08);
    System.out.println("aglea");
    
  }                      
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("isfinishedidle");
    tempAlgae.setAlgaePivotSpeed(.08);
    return true;
    }
  }