// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * Trajectory Constants
 * Gains are calculated by the SysID tool from WPILib. 
 * SysID settings: 
 * Analysis Type: Drivetrain
 * Motor Controllers
 * Pair 0: SparkMax, Left: 1, Right: 3; right inverted
 * Pair 1: SparkMax, Left: 2, Right: 4; right inverted
 * Encoder Selection: Encoder Port
 * Gyro: Navx; MXP port
 * Encoder parameters: counts per revolution: 1, 10.71:1 gearing
 * Logger parameters: Units per rotation: .4788
 * 
 * In Feedback analysis: Gain preset is Rev Brushless Encoder Port
 * Convert gains to encoder counts is OFF. 
 * 
 * Leave all other settings at default. Run the 4 tests for both drivetrain and angular drivetrain. 
 */
    //Feedforward constants from linear tests
    public static final double ks = 0.19098;//volts
    public static final double kv = 2.7743;//volt seconds per meter
    public static final double ka = 0.56749;//volt seconds squared per meter

    //Track width from angular tests
    public static final double trackWidth = 0.59328; //meters

    //Proportional constant from linear tests
    public static final double kP = 0.0014723; //voltage per velocity

    //Object to tell trajectory how to vary speed for track width
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidth); 

    //Ramsete constants recommended by WPILib as "consistent" for all sorts of robots. 
    //Results of changes unknown. 
    public static final double kRamseteB = 2;
    public static final double kRamseteZ = .7;
}
