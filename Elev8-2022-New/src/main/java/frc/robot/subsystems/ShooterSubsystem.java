// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;
  private final CANSparkMax feederMotor;
  private final Servo FeederServo;


  public ShooterSubsystem() {


    //FWL = new WPI_TalonSRX(Constants.FWL_port);
    //FWR = new WPI_TalonSRX(Constants.FWR_port);
    //flyWheel = new MotorControllerGroup(FWL, FWR);
    shooterMotor = new CANSparkMax(Constants.ShooterPort, MotorType.kBrushless); //Have to check whether its brushless or brushed
    feederMotor = new CANSparkMax(Constants.FeederPort, MotorType.kBrushless);
    FeederServo = new Servo(Constants.FeederServoPort);

    //HoodL = new Servo(Constants.HoodL_port);
    //HoodR = new Servo(Constants.HoodR_port);
    //Hood = new MotorControllerGroup(HoodL, HoodR);




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void shootTime(double speed, double rampupTime) {
    double now = Timer.getFPGATimestamp();

    while (now < rampupTime) {
      if (now == rampupTime/3) {
        shooterMotor.set(speed/3);
      }
      if (now == rampupTime/2) {
        shooterMotor.set(speed/2);
      }
    }
    shooterMotor.set(speed);
    
  }

  double integral = 0;
  double prevError = 0;
  public void shootPID(double sSpeed, double fSpeed) {
    double error = sSpeed - shooterMotor.get();

    double derivative = error - prevError;
    prevError = error;

    integral = error + integral;
    
    double correction = error*Constants.kPShoot + derivative*Constants.kDShoot + integral*Constants.kIShoot;
    shooterMotor.set(sSpeed + correction);

    feederMotor.set(fSpeed);
  }

  public void shootRaw(double speed) {
    shooterMotor.set(speed);
  }

  public void setServo(double power){
    FeederServo.set(power);
  }
}
