package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase
{
  // FIRST
  
  WPI_TalonFX hangerLeftTalonFX;
  WPI_TalonFX hangerRightTalonFX;
  TalonSRX hangerOuterArm;

  //Second

  CANSparkMax hangerLeftNeo;
  CANSparkMax hangerRightNeo;

  TalonSRX hangerInnerArm;


  public HangerSubsystem()
  {

    hangerLeftTalonFX = new WPI_TalonFX(Constants.leftTalonFXPort);
    hangerRightTalonFX = new WPI_TalonFX(Constants.rightTalonFXPort);
  
    hangerOuterArm = new TalonSRX(Constants.outerHangerArmPort);


    hangerLeftNeo = new CANSparkMax(Constants.hangerLeftNeoPort, MotorType.kBrushless);
    hangerRightNeo = new CANSparkMax(Constants.hangerLeftNeoPort, MotorType.kBrushless);

    hangerInnerArm = new TalonSRX(Constants.innerHangerArmPort);

  }

  @Override
  public void periodic()
  {
    
  }
}
