package frc.robot.commands.Hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class ReachFirstRungCommand extends CommandBase {
    HangerSubsystem hangerSubsystem;
    double firstRungSpeed;

  public ReachFirstRungCommand(HangerSubsystem hangerSubsystem, double firstRungSpeed) {
    this.hangerSubsystem = hangerSubsystem;
    this.firstRungSpeed = firstRungSpeed;
    addRequirements(hangerSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    hangerSubsystem.reachFirstRung(firstRungSpeed);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
