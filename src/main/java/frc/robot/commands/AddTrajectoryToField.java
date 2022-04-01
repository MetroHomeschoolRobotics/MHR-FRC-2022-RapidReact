// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.TrajectoryHelper;

public class AddTrajectoryToField extends CommandBase {
  /** Creates a new AddTrajectoryToField. */
  Field2d _field;
  Trajectory _Trajectory;
  public AddTrajectoryToField(Field2d field, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    _field = field;
    _Trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrajectoryHelper.putTrajectoryOnField(_Trajectory, _field);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
