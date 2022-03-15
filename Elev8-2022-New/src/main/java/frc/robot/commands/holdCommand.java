// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangerSubsystem;

public class holdCommand extends CommandBase {
  /** Creates a new holdCommand. */
  HangerSubsystem hangerSubsystem;
  TalonSRX motor;
  public holdCommand(HangerSubsystem hangerSubsystem, TalonSRX motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hangerSubsystem = hangerSubsystem;
    this.motor = motor;
    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangerSubsystem.holdPosition(motor);
    
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
