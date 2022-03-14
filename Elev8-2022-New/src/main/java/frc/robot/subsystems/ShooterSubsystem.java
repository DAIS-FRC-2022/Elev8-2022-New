// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ShooterSubsystem. */
<<<<<<< Updated upstream
  //private final WPI_TalonSRX FWL;
  //private final WPI_TalonSRX FWR;
  //private final MotorControllerGroup flyWheel;
  private final CANSparkMax Shooter;
  //private final Servo HoodL;
  //private final Servo HoodR;
  //private final MotorControllerGroup Hood;


  public ShooterSubsystem() {


    //FWL = new WPI_TalonSRX(Constants.FWL_port);
    //FWR = new WPI_TalonSRX(Constants.FWR_port);
    //flyWheel = new MotorControllerGroup(FWL, FWR);
    Shooter = new CANSparkMax(Constants.Shooter_port, MotorType.kBrushless); //Have to check whether its brushless or brushed

    //HoodL = new Servo(Constants.HoodL_port);
    //HoodR = new Servo(Constants.HoodR_port);
    //Hood = new MotorControllerGroup(HoodL, HoodR);

=======
  private final CANSparkMax shooterMotor;
  private final SparkMaxPIDController shootPIDController;
  public static RelativeEncoder shootEncoder;
  public static Servo feedServo;

  public ShooterSubsystem() {

    // FWL = new WPI_TalonSRX(Constants.FWL_port);
    // FWR = new WPI_TalonSRX(Constants.FWR_port);
    // flyWheel = new MotorControllerGroup(FWL, FWR);
    shooterMotor = new CANSparkMax(Constants.Shooter_port, MotorType.kBrushless); // Have to check whether its brushless
                                                                                  // or brushed
    this.shooterMotor.restoreFactoryDefaults();
    this.shootPIDController = this.shooterMotor.getPIDController();
    ShooterSubsystem.shootEncoder = this.shooterMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    this.feedServo = new Servo(0);

    this.shootPIDController.setP(0.00012);
    this.shootPIDController.setI(3e-8);
    this.shootPIDController.setD(1.2);
    this.shootPIDController.setIZone(0);
    this.shootPIDController.setFF(0.00017);
    this.shootPIDController.setOutputRange(-1, 1);
    // HoodL = new Servo(Constants.HoodL_port);
    // HoodR = new Servo(Constants.HoodR_port);
    // Hood = new MotorControllerGroup(HoodL, HoodR);
>>>>>>> Stashed changes

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //shooterMotor.set(0.5);
    



  }

<<<<<<< Updated upstream
  public void shoot(double speed, double seconds)
  {
    //FWR.setInverted(false);

    double startTime = System.currentTimeMillis();
    while ((startTime - System.currentTimeMillis())/1000 <= seconds)
    {
      Shooter.set(speed);
    }
    Shooter.set(0);
    
=======
  public void shootTime(double speed, double rampupTime) {
    double now = Timer.getFPGATimestamp();

    while (now < rampupTime) {
      if (now == rampupTime / 3) {
        shooterMotor.set(speed / 3);
      }
      if (now == rampupTime / 2) {
        shooterMotor.set(speed / 2);
      }
    }
    shooterMotor.set(speed);

>>>>>>> Stashed changes
  }

  double integral = 0;
  double prevError = 0;
<<<<<<< Updated upstream
  public void shootPID(double speed, double seconds)
  {
    double startTime = System.currentTimeMillis();
    while ((startTime - System.currentTimeMillis())/1000 <= seconds)
    {
      double error = speed - Shooter.get();
=======

  public void shootPID(double speed, double seconds) {
    double now = Timer.getFPGATimestamp();
    while (now <= seconds) {
      double error = speed - shooterMotor.get();
>>>>>>> Stashed changes
      double derivative = error - prevError;
      prevError = error;
      double correction = error * Constants.kPShoot + derivative * Constants.kDShoot + integral * Constants.kIShoot;
      integral = error + integral;
      Shooter.set(speed + correction);
    }
    Shooter.set(0);
  }

<<<<<<< Updated upstream
  /*public void setHood (double angle)
=======
  public void shootRaw(double speed) {
    shooterMotor.set(speed);
  }

  public void shootProp(double rpm) {
    double speedmotor = shootEncoder.getVelocity();
    if (Math.abs(speedmotor) < 1500) {

        this.shootPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        
        // SmartDashboard.putNumber("POSITION",
        // ShooterSubsystem.shootEncoder.getPosition());
        SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
      }
    else
      {this.shooterMotor.set(0);}
  }

  public void servoMove(double angle)
>>>>>>> Stashed changes
  {
    feedServo.setAngle(angle);
  }
  /*
   * public void setHood (double angle)
   * {
   * HoodL.setAngle(angle);
   * HoodR.setAngle(angle);
   * }
   */

}
