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
    //-------------------------//
    //-------TRAJECTORY--------//
    //-------CONSTANTS---------//
    //-------------------------//

    //Feedforward constants from linear tests
    public static final double ks = .16282;//volts
    public static final double kv = 2.8616;//volt seconds per meter
    public static final double ka = 0.48652;//volt seconds squared per meter

    //Feedforward constants from angular tests
    public static final double ksAngular = .30662;//volts
    public static final double kvAngular = 204.87;//volt seconds per meter
    public static final double kaAngular = 19.645;//volt seconds squared per meter

    //Track width from angular tests
    public static final double trackWidth = .60508; //meters

    //Proportional constant from linear tests
    public static final double kP = 3.6888E-08; //voltage per velocity

    //Proportional constant from angular tests
    public static final double kPAngular = 0.0042148; //voltage per velocity

    //Object to tell trajectory how to vary speed for track width
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidth); 

    //Max velocity and acceleration we want the program to run
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    //Ramsete constants recommended by WPILib as "consistent" for all sorts of robots. 
    //Results of changes unknown. 
    public static final double kRamseteB = 2;
    public static final double kRamseteZ = .7;
}
