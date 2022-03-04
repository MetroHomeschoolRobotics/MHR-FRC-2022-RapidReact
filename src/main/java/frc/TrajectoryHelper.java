// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetOdometry;

/** Add your docs here. */
public class TrajectoryHelper {
  /**
   * Generates a trajectory from the given waypoints and settings. 
   * @param start The starting pose.
   * @param waypoints The interior waypoints.
   * @param end The ending pose.
   * @param reversed If the trajectory should be run backwards.
   * @param maxVelocity The max speed the robot should go. 
   * @param maxAccel The max acceleration you want the robot to go. 
   * @param startVelocity The speed the robot starts the trajectory at. 
   * @param endVelocity The speed the robot finishes the trajectory at. 
   * @param maxVoltage The maximum voltage the motors can use; will affect acceleration. 
   * @return The generated trajectory.
   */

    public final static Trajectory generateTrajectory(Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed, double maxVelocity, double maxAccel, double startVelocity, double endVelocity, double maxVoltage) {
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka), Constants.kDriveKinematics, maxVoltage);
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);
        config.setStartVelocity(startVelocity);
        config.setEndVelocity(endVelocity);
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }
  /**
   * Creates a command that runs the given trajectory. 
   * @param trajectoryToFollow The trajectory the robot should follow. 
   * @return A RamseteCommand that follows the specified trajectory.
   */
    public final static Command createTrajectoryCommand(Trajectory _trajectoryToFollow) {
        var leftController = new PIDController(Constants.kP, 0, 0);
        var rightController = new PIDController(Constants.kP, 0, 0);
        //RobotContainer.s_drivetrain.getField2d().getObject("traj").setTrajectory(_trajectoryToFollow);
        RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZ);
        RamseteCommand ramseteCommand = new RamseteCommand(
          _trajectoryToFollow,
          RobotContainer.s_drivetrain::getPose,
          ramseteController,
          new SimpleMotorFeedforward(Constants.ks, Constants.kv,Constants.ka),
          Constants.kDriveKinematics,
          RobotContainer.s_drivetrain::getWheelSpeeds,
          leftController,
          rightController,
          (leftVolts, rightVolts) -> {
            RobotContainer.s_drivetrain.tankDriveVolts(leftVolts, rightVolts);
          },
          RobotContainer.s_drivetrain);
          RobotContainer.s_drivetrain.resetOdometry(_trajectoryToFollow.getInitialPose());
          return new SequentialCommandGroup(new ResetOdometry(_trajectoryToFollow.getInitialPose(), RobotContainer.s_drivetrain), 
          ramseteCommand);
    }
    
}
