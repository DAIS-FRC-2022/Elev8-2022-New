// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import com.revrobotics.SparkMaxRelativeEncoder;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;
  private final CANSparkMax feederMotor;
  private final SparkMaxPIDController shootPIDController;
  private final Servo servo;
  public static RelativeEncoder shootEncoder;


  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(Constants.ShooterPort, MotorType.kBrushless); //Have to check whether its brushless or brushed
    feederMotor = new CANSparkMax(Constants.FeederPort, MotorType.kBrushless);

    servo = new Servo(Constants.feederServoChannel);
    this.shootPIDController = this.shooterMotor.getPIDController();
    ShooterSubsystem.shootEncoder = this.shooterMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    this.shootPIDController.setP(0.0012);
    this.shootPIDController.setI(0);
    this.shootPIDController.setD(0);
    this.shootPIDController.setIZone(0);
    this.shootPIDController.setFF(0.00017);
    this.shootPIDController.setOutputRange(-1, 1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterMotor.set(0.2);
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

  public void shootRaw(double sspeed){//, double fspeed) {
    shooterMotor.set(sspeed);
    //feederMotor.set(fspeed);
  }

  public void feed(double speed) {
    feederMotor.set(speed);
  }

  public void setServo(double pos){
    servo.set(pos);
  }

  public void shootProp(double rpm)
  {
    // double speedmotor = shootEncoder.getVelocity();
    // if (Math.abs(speedmotor) < 1500) {
    //   this.shootPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    //   // SmartDashboard.putNumber("POSITION",
    //   // ShooterSubsystem.shootEncoder.getPosition());
    //   SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
    // } 
    // else
    // {
    //   this.shooterMotor.set(0);
    // }
    this.shootPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
      // SmartDashboard.putNumber("POSITION",
      // ShooterSubsystem.shootEncoder.getPosition());
    //SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
  }
}
