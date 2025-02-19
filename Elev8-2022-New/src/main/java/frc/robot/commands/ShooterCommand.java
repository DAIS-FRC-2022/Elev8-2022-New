// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  double shooterSpeed, servoAngle;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed, double servoAngle){
    this.shooterSubsystem = shooterSubsystem;
    this.shooterSpeed = shooterSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooterSubsystem.shootTime(speed, 1.5);
    // shooterSubsystem.shootRaw(sSpeed);
    shooterSubsystem.shootPID(shooterSpeed);
    shooterSubsystem.setServo(servoAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
