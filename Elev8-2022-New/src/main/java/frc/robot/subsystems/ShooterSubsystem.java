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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase
{

  private final CANSparkMax shooterMotor;
  private final CANSparkMax feederMotor;
  private final SparkMaxPIDController shootPIDController;
  private final Servo FeederServo;
  public static RelativeEncoder shootEncoder;


  public ShooterSubsystem()
  {

    shooterMotor = new CANSparkMax(Constants.ShooterPort, MotorType.kBrushless); //Have to check whether its brushless or brushed
    feederMotor = new CANSparkMax(Constants.FeederPort, MotorType.kBrushless);
    FeederServo = new Servo(Constants.FeederServoPort);

    this.shootPIDController = this.shooterMotor.getPIDController();

    ShooterSubsystem.shootEncoder = this.shooterMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);



    this.shootPIDController.setP(0.00012);
    this.shootPIDController.setI(3e-8);
    this.shootPIDController.setD(1.2);
    this.shootPIDController.setIZone(0);
    this.shootPIDController.setFF(0.00017);
    this.shootPIDController.setOutputRange(-1, 1);

  }

  @Override
  public void periodic() {

   // shooterMotor.set(0.5);
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

  public void shootRaw(double sspeed){//, double fspeed) {
    shooterMotor.set(sspeed);
    //feederMotor.set(fspeed);
  }

  public void feed(double speed)
  {
    feederMotor.set(speed);
  }

  public void setServo(double power){
    FeederServo.set(power);
  }

  public void shootProp(double pow)

  {
    double speedmotor = shootEncoder.getVelocity();
    if (Math.abs(speedmotor) < 1500)
    {
      this.shootPIDController.setReference(pow*6000, CANSparkMax.ControlType.kVelocity);
      // SmartDashboard.putNumber("POSITION",
      // ShooterSubsystem.shootEncoder.getPosition());
      SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
    }
    else
    {
      this.shooterMotor.set(0);
    }

  }
}
