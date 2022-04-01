// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberWinch;

public class RunClimberTillLimit extends CommandBase {
  /** Creates a new RunArmTillLimit. */
  ClimberWinch climber;
  double _speed;
  public RunClimberTillLimit(ClimberWinch s_climber, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = s_climber;
    _speed = speed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_speed<0) {
    if(!climber.getLowerLimitSwitch()) {
      climber.setClimberWinchMotor(_speed);
    } else {
      climber.setClimberWinchMotor(-_speed);
    }
  } else if(_speed>0) {
    if(!climber.getUpperLimitSwitch()) {
      climber.setClimberWinchMotor(_speed);
    } else {
      climber.setClimberWinchMotor(-_speed);
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_speed<0) {
      return climber.getLowerLimitSwitch();
    } else if(_speed>0) {
      return climber.getUpperLimitSwitch();
    } else {
      return true;
    }
  }
}
