// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  /** Creates a new HangerSubsystem. */
  private final CANSparkMax leftHanger;
  private final CANSparkMax rightHanger;
  public final WPI_TalonSRX piggyLeft;
  public final WPI_TalonSRX piggyRight;
  private final MotorControllerGroup hanger;
  private final MotorControllerGroup piggies;

  public HangerSubsystem() {
    leftHanger = new CANSparkMax(Constants.leftHangerPort, MotorType.kBrushless);
    rightHanger = new CANSparkMax(Constants.rightHangerPort, MotorType.kBrushless);
    hanger = new MotorControllerGroup(leftHanger, rightHanger);
    piggyLeft = new WPI_TalonSRX(Constants.piggyLeftPort);
    piggyRight = new WPI_TalonSRX(Constants.piggyRightPort);
    piggies = new MotorControllerGroup(piggyLeft, piggyRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void FXMove(double pow)
  {
    rightHanger.setInverted(true);
    hanger.set(pow);
  }

  public void NeoMove(double pow)
  {
    rightHanger.setInverted(true);
    hanger.set(pow);
    //piggyLeft.setSelectedSensorPosition(sensorPos);
    //piggyLeft.setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs)
  }



  // idk what this is
  public void holdPosition(WPI_TalonSRX motor)
  {
    motor.setSelectedSensorPosition(motor.getSelectedSensorPosition(0), 0, 0);
  }


}
