// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTeleop extends CommandBase {
  /** Creates a new DriveTeleop. */
  private Drivetrain _drivetrain;
  private XboxController _driverController;
  private double forward;
  private double turn;
  private SlewRateLimiter filter = new SlewRateLimiter(0.5);
  public DriveTeleop(Drivetrain drivetrain, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    _driverController = driverController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward = _driverController.getLeftY();
    turn = _driverController.getLeftX();
    forward=Math.pow(forward,2);
    turn = Math.pow(turn,2);    //Inputs are squared, allowing for finer control with the same max input
    if(!_driverController.getRightBumper()) {
      forward = filter.calculate(forward);//A slew rate limiter is used to cap acceleration. (if a button is not held)
    }
    _drivetrain.move(turn, forward, _driverController.getLeftBumper());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.move(0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
