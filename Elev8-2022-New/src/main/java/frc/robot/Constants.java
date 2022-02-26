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

    /*PORTS*/
    public static final int FR_port = 21;
    public static final int FL_port = 11;
    public static final int BR_port = 22; 
    public static final int BL_port = 12;
    public static final int Shooter_port = 1;
    public static final int Intake = 7;


    public static double arcadeMaxSpeed = 0.6;
    public static double maxSpeed = 0.3;
    public static double minSpeed = 0.1;
    public static double deadband = 0.02;

    // Encoders
    public static double encoderScale = 0.001425;
    public static double rightScale = 0.25;

    public static final double G = 9.81;

    public static double kPTurn = 0.0085;
    public static double kPDist = 0.21;


    public static double navxScale = 1.1;

    public static double goalHt = 2.6416; //set this to the actual goal height (in metres preferably)
    public static double limelightAngle = 30d;
    public static double ShotConstant = 0.001;
    public static double kPShoot = 0.001;
    public static double kIShoot = 0.001;    
    public static double kDShoot = 0.001;
    // kP, kI, kD to be tuned

    
    public static final int shootAssistButtonNum = 5;
}
