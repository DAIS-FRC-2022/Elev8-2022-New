// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax Intake;
  private final CANSparkMax Feeder;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    Intake = new CANSparkMax(Constants.IntakePort, MotorType.kBrushless);
    Feeder = new CANSparkMax(Constants.FeederPort, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake(double Ispeed)
  {
    Intake.set(Ispeed);
  }

  public void feeder(double Fspeed)
  {
    Feeder.set(Fspeed);
  }
}
