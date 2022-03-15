// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  /** Creates a new HangerSubsystem. */
  private final CANSparkMax leftFirstRung;
  private final CANSparkMax rightFirstRung;
  private final MotorControllerGroup firstRung;

  private final WPI_TalonFX leftSecondRung;
  private final WPI_TalonFX rightSecondRung;
  private final MotorControllerGroup secondRung;

  public final WPI_TalonSRX piggyLeft;
  public final WPI_TalonSRX piggyRight;
  private final MotorControllerGroup piggies;

  public HangerSubsystem() {
    leftFirstRung = new CANSparkMax(Constants.leftFirstRungPort, MotorType.kBrushless);
    rightFirstRung = new CANSparkMax(Constants.rightFirstRungPort, MotorType.kBrushless);
    firstRung = new MotorControllerGroup(leftFirstRung, rightFirstRung);

    leftSecondRung = new WPI_TalonFX(Constants.leftSecondRungPort);
    rightSecondRung = new WPI_TalonFX(Constants.rightSecondRungPort);
    secondRung = new MotorControllerGroup(leftSecondRung, rightSecondRung);

    piggyLeft = new WPI_TalonSRX(Constants.piggyLeftPort);
    piggyRight = new WPI_TalonSRX(Constants.piggyRightPort);
    piggies = new MotorControllerGroup(piggyLeft, piggyRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reachFirstRung(double pow)
  {
    rightFirstRung.setInverted(true);
    firstRung.set(pow);
  }

  public void reachSecondRung(double pow)
  {
    rightSecondRung.setInverted(true);
    secondRung.set(pow);
  }

  // idk what this is
  public void holdPosition(WPI_TalonSRX motor)
  {
    motor.setSelectedSensorPosition(motor.getSelectedSensorPosition(0), 0, 0);
  }

}
