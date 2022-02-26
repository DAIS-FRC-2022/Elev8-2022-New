// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor;


  public ShooterSubsystem() {


    //FWL = new WPI_TalonSRX(Constants.FWL_port);
    //FWR = new WPI_TalonSRX(Constants.FWR_port);
    //flyWheel = new MotorControllerGroup(FWL, FWR);
    shooterMotor = new CANSparkMax(Constants.Shooter_port, MotorType.kBrushless); //Have to check whether its brushless or brushed

    //HoodL = new Servo(Constants.HoodL_port);
    //HoodR = new Servo(Constants.HoodR_port);
    //Hood = new MotorControllerGroup(HoodL, HoodR);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void shootTime(double speed, double seconds)
  {
    double now = Timer.getFPGATimestamp();

    while (now < 0.9) {
      if (now == 0.3) {
        shooterMotor.set(speed/3);
      }
      if (now == 0.6) {
        shooterMotor.set(speed/2);
      }
    }
    shooterMotor.set(speed);
    
  }

  double integral = 0;
  double prevError = 0;
  public void shootPID(double speed, double seconds)
  {
    double startTime = System.currentTimeMillis();
    while ((startTime - System.currentTimeMillis())/1000 <= seconds)
    {
      double error = speed - shooterMotor.get();
      double derivative = error - prevError;
      prevError = error;
      double correction = error*Constants.kPShoot + derivative*Constants.kDShoot + integral*Constants.kIShoot;
      integral = error + integral;
      shooterMotor.set(speed + correction);
    }
    shooterMotor.set(0);
  }

  public void shootRaw(double speed) {
    shooterMotor.set(speed);
  }

  /*public void setHood (double angle)
  {
    HoodL.setAngle(angle);
    HoodR.setAngle(angle);
  }*/

}
