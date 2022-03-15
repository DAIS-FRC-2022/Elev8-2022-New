package frc.robot.commands.Hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class ReachSecondRungCommand extends CommandBase {
    HangerSubsystem hangerSubsystem;
    double secondRungSpeed;

  public ReachSecondRungCommand(HangerSubsystem hangerSubsystem, double secondRungSpeed) {
    this.hangerSubsystem = hangerSubsystem;
    this.secondRungSpeed = secondRungSpeed;
    addRequirements(hangerSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    hangerSubsystem.reachSecondRung(secondRungSpeed);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}