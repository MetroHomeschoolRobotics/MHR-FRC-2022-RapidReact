// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberWinch;

public class WinchClimber extends CommandBase {
  /** Creates a new WinchClimber. */
  ClimberWinch climber;
  XboxController controller;
  XboxController controller2;
  public WinchClimber(ClimberWinch _climber, XboxController _controller, XboxController _controller2) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = _climber;
    controller = _controller;
    controller2 = _controller2;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getRightY()+controller2.getRightY();
    if(Math.abs(speed)<.1) {
      speed = 0;
    }
    climber.setClimberWinchMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
