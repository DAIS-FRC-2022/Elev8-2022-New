// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class GoalSeekingCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  LimelightShoot limelightShoot;
  boolean tv;
  /** Creates a new GoalSeekingCommand. */
  public GoalSeekingCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.navx.reset();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    driveSubsystem.drive(Constants.maxSpeed, -Constants.maxSpeed); //can change speeds if there is overshoot
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.tv;
  }
}
