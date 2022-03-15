// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HangerSubsystem;

public class TestCommand extends CommandBase {
  /** Creates a new TestCommand. */
  HangerSubsystem hangerSubsystem;
  double pow;
  WPI_TalonSRX motor;
  public TestCommand(HangerSubsystem hangerSubsystem, double pow, WPI_TalonSRX motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hangerSubsystem = hangerSubsystem;
    this.pow = pow;
    this.motor = motor;
    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.set(ControlMode.PercentOutput, this.pow);
    //motor.setSelectedSensorPosition(180, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
