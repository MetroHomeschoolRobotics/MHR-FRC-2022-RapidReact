// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.TrajectoryHelper;

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

    public static final double robotLengthMeters = Units.inchesToMeters(32+(3.25*2));

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

    public static double maxForwardVel = 8;
    public static double maxForwardAccel = 5;
    public static double maxForwardCAccel = 3;

    public static double maxBackwardVel = 8;
    public static double maxBackwardAccel = 5;
    public static double maxBackwardCAccel = 3;

    //Points on the Field
    private static final Pose2d rightFender = new Pose2d(7.78, 2.87, Rotation2d.fromDegrees(-111));
    private static final Pose2d leftFender = new Pose2d(7.01, 4.64, Rotation2d.fromDegrees(159));
    
    private static final Pose2d intakingRLBall = new Pose2d(5.64, 1.93, Rotation2d.fromDegrees(-180));
    private static final Pose2d intakingRRBall = new Pose2d(7.68, 0.84, Rotation2d.fromDegrees(-90));
    private static final Pose2d intakingLBall = new Pose2d(5.3, 5.96, Rotation2d.fromDegrees(135));

    private static final Pose2d intakingTerminalBalls = new Pose2d(1.71, 1.59, Rotation2d.fromDegrees(-135));

    public static final Trajectory intakeBothRightFenderBalls = TrajectoryHelper.generateTrajectory(rightFender, List.of(intakingRLBall.getTranslation()), intakingRRBall, false, 3, 2, 1, 0, 0, 7);
    public static final Trajectory intakeRRBall = TrajectoryHelper.generateTrajectory(rightFender, List.of(), intakingRRBall, false, maxForwardVel, maxForwardAccel, maxForwardCAccel, 0, 0, 7);
    public static final Trajectory intakeRLBall = TrajectoryHelper.generateTrajectory(rightFender, List.of(), intakingRLBall, false, maxForwardVel, maxForwardAccel, maxForwardCAccel, 0, 0, 7);
    public static final Trajectory intakeLBall = TrajectoryHelper.generateTrajectory(leftFender, List.of(), intakingLBall, false, maxForwardVel, maxForwardAccel, maxForwardCAccel, 0, 0, 7);
    public static final Trajectory toTerminalFromLFender = TrajectoryHelper.generateTrajectory(leftFender, List.of(), intakingTerminalBalls, false, maxForwardVel, maxForwardAccel, maxForwardCAccel, 0, 0, 7);

    public static final Trajectory backToFenderFromRLBall = TrajectoryHelper.generateTrajectory(intakingRLBall, List.of(), rightFender, true, maxBackwardVel, maxBackwardAccel, maxBackwardCAccel, 0, 1, 7);
    public static final Trajectory backToFenderFromRRBall = TrajectoryHelper.generateTrajectory(intakingRRBall, List.of(), rightFender, true, maxBackwardVel, maxBackwardAccel, maxBackwardCAccel, 0, 1, 7);
    public static final Trajectory backToLFenderFromLBall = TrajectoryHelper.generateTrajectory(intakingLBall, List.of(), leftFender, true, maxBackwardVel, maxBackwardAccel, maxBackwardCAccel, 0, 1, 7);
    public static final Trajectory backToLFenderFromTerminal = TrajectoryHelper.generateTrajectory(intakingTerminalBalls, List.of(), leftFender, true, maxBackwardVel, maxBackwardAccel, maxBackwardCAccel, 0, 1, 7);
    public static final Trajectory backToRFenderFromTerminal = TrajectoryHelper.generateTrajectory(intakingTerminalBalls, List.of(), rightFender, true, maxBackwardVel, maxBackwardAccel, maxBackwardCAccel, 0, 1, 7);

}
