// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  /** Creates a new HangerSubsystem. */
  private final WPI_TalonFX leftFirstRung;
  private final WPI_TalonFX rightFirstRung;
  private final MotorControllerGroup firstRung;

  private final CANSparkMax leftSecondRung;
  private final CANSparkMax rightSecondRung;
  private final MotorControllerGroup secondRung;

  private final RelativeEncoder leftSecondRungEncoder;
  private final RelativeEncoder rightSecondRungEncoder;

  public final WPI_TalonSRX pgLeft;
  public final WPI_TalonSRX pgRight;
  private final MotorControllerGroup pgs;

  public HangerSubsystem() {
    leftFirstRung = new WPI_TalonFX(Constants.leftFirstRungPort);
    rightFirstRung = new WPI_TalonFX(Constants.rightFirstRungPort);
    firstRung = new MotorControllerGroup(leftFirstRung, rightFirstRung);

    leftSecondRung = new CANSparkMax(Constants.leftSecondRungPort, MotorType.kBrushless);
    rightSecondRung = new CANSparkMax(Constants.rightSecondRungPort, MotorType.kBrushless);
    secondRung = new MotorControllerGroup(leftSecondRung, rightSecondRung);
    leftSecondRungEncoder = leftSecondRung.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    rightSecondRungEncoder = rightSecondRung.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    pgLeft = new WPI_TalonSRX(Constants.piggyLeftPort);
    pgRight = new WPI_TalonSRX(Constants.piggyRightPort);
    pgs = new MotorControllerGroup(pgLeft, pgRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reachFirstRung(double pow) {
    rightFirstRung.setInverted(true);
    firstRung.set(pow);
  }

  public void reachSecondRung(double pow) {
    rightSecondRung.setInverted(true);
    secondRung.set(pow);
  }

  public double getEncoderFirstRung() {
    return (leftSecondRungEncoder.getPosition() + rightSecondRungEncoder.getPosition())/2;
  }

  public double getEncoderSecondRung() {
    return (leftSecondRung.getEncoder().getPosition() + rightSecondRung.getEncoder().getPosition())/2;
  }

}
