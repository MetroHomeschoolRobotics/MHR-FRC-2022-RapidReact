// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

    public final static Trajectory generateTrajectory(Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed, double maxVelocity, double maxAccel, double maxCentripAccel, double startVelocity, double endVelocity, double maxVoltage) {
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka), Constants.kDriveKinematics, maxVoltage);
        CentripetalAccelerationConstraint autoTurningConstraint = new CentripetalAccelerationConstraint(maxCentripAccel);
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel).setKinematics(Constants.kDriveKinematics);
        config.addConstraint(autoVoltageConstraint);
        config.addConstraint(autoTurningConstraint);
        config.setStartVelocity(startVelocity);
        config.setEndVelocity(endVelocity);
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }
    public final static Trajectory generateFromPathPlanner(String trajectoryJSON) {
      trajectoryJSON = "pathplanner/generatedJSON/"+trajectoryJSON+".wpilib.json";
      Trajectory trajectory;
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        trajectory= TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), new Pose2d(), config);
        }
        return trajectory;
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
          return ramseteCommand;
    }
    
}
