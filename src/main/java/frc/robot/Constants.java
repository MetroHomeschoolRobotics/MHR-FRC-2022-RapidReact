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
    public static final double ks = 0.17502;//volts
    public static final double kv = 2.8214;//volt seconds per meter
    public static final double ka = 0.34693;//volt seconds squared per meter

    public static final double ksAngular = .30662;//volts
    public static final double kvAngular = 204.87;
    public static final double kaAngular = 19.645;

    public static final double trackWidth = .66996; //m

    public static final double kP = 4.2339E-05;

    public static final double kPAngular = 0.0042148;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidth);

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZ = .7;
}
