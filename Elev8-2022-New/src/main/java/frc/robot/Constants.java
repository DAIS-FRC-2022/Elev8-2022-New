// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Motor Ports
    public static final int FR_port = 21;
    public static final int FL_port = 11;
    public static final int BR_port = 22; 
    public static final int BL_port = 12;

    // Other susbsytem ports
    public static final int IntakePort = 1;
    public static final int FeederPort = 2;
    public static final int ShooterPort = 3;
    public static final int feederServoChannel = 0;

    // Hanger ports - WRONG
    public static final int rightFirstRungPort = 5; // falcon
    public static final int leftFirstRungPort = 6; // falcon

    public static final int rightSecondRungPort = 5; // neo
    public static final int leftSecondRungPort = 6; // neo

    public static final int pgRightPort = 14;
    public static final int pgLeftPort = 15;

    public static double arcadeMaxSpeed = 0.5;
    public static double maxSpeed = 0.8;
    public static double minSpeed = 0.1;
    public static double deadband = 0.2;

    // Encoders
    public static double encoderScale = 0.03;
    public static double rightScale = 0.25;
    public static double gearRatio = 7.31;

    public static final double G = 9.81;

    public static double kPTurn = 0.0085;
    public static double kPDist = 0.21;

    public static double wheel2wheelDist = 23.75;


    public static double navxScale = 1.1;

    public static double goalHt = 2.6416; //set this to the actual goal height (in metres preferably)
    public static double limelightAngle = 30d;
    public static double ShotConstant = 0.001;
    
    public static double kPShoot = 0.01;
    public static double kIShoot = 0;    
    public static double kDShoot = 1;
    // kP, kI, kD to be tuned
    
    public static double wheelDia = 0.152d;
    public static double rpm = 93.37;
    public static double realMaxSpeed = rpm/60*Math.PI*wheelDia;
    public static double Tperiod = 1d;

    public static final int intakeButtonNum = 2;
    public static final int feederButtonNum = 12;
    public static final int shooterButtonNum = 1;

    public static final int reachFirstRungButton = 100;
    public static final int reachSecondRungButton = 111;
}
