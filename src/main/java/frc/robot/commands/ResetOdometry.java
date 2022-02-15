// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends InstantCommand {
  public ResetOdometry(Pose2d poseToSet, Drivetrain drivetrain) {
    //resets drivetrain odometry to a specific pose. 
    //Used in the RobotContainer's createTrajectoryCommand(Trajectory) method to zero robot trajectory before the trajectory starts. 
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain.resetOdometry(poseToSet);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
