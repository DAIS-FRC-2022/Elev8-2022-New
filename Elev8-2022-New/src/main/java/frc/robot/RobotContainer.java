package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Hanger.ReachFirstRungCommand;
import frc.robot.commands.Hanger.ReachSecondRungCommand;
import frc.robot.commands.Limelight.*;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
 
  // The robot's subsystems and commands are defined here...
 
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final HangerSubsystem hangerSubsystem = new HangerSubsystem();

  // Commands
  
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);


  // IO Devices
  
  public static Joystick joy1 = new Joystick(0);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);
  }

  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(joy1, Constants.intakeButtonNum);
    intakeButton.whenActive(new IntakeCommand(intakeSubsystem, -0.5));
    intakeButton.whenReleased(new IntakeCommand(intakeSubsystem, 0));

    // JoystickButton feederServoButton = new JoystickButton(joy1, Constants.shooterButtonNum);
    // feederServoButton.whenActive(new ShooterCommand(shooterSubsystem, 0.8, 0));
    // feederServoButton.whenReleased(new ShooterCommand(shooterSubsystem, 0.2, 0));

    JoystickButton feederButton = new JoystickButton(joy1, Constants.feederButtonNum);
    feederButton.whenActive(new FeederCommand(shooterSubsystem, 0.5));
    feederButton.whenReleased(new FeederCommand(shooterSubsystem, 0));

    JoystickButton reachFirstRungButton = new JoystickButton(joy1, Constants.reachFirstRungButton);
    reachFirstRungButton.whenActive(new ReachFirstRungCommand(hangerSubsystem, 0.2));
    reachFirstRungButton.whenReleased(new ReachFirstRungCommand(hangerSubsystem, 0));

    JoystickButton reachSecondRungButton = new JoystickButton(joy1, Constants.reachSecondRungButton);
    reachSecondRungButton.whenActive(new ReachSecondRungCommand(hangerSubsystem, 0.2));
    reachSecondRungButton.whenReleased(new ReachSecondRungCommand(hangerSubsystem, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new MoveByDistanceCommand(driveSubsystem, 1);
  }

  public static double getY(Joystick joy, double deadband) {
    double value = -1 * joy.getY();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

  public static double getZ(Joystick joy, double deadband) {
    double value = joy.getZ();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

}
