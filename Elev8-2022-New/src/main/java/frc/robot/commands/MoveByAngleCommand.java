package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class MoveByAngleCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double setpoint, error;

  public MoveByAngleCommand(DriveSubsystem driveSubsystem, double setpoint)
  {
    this.driveSubsystem = driveSubsystem;
    this.setpoint = setpoint;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.navx.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    this.error = this.setpoint - (RobotContainer.navx.getYaw());

    
    double correction = this.error * Constants.kPTurn;
    this.driveSubsystem.moveByAngle(correction);
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished()
  {
    return (Math.abs(this.error) <= /*Math.max(10.00d, (*/this.setpoint * 0.15);
  }
}