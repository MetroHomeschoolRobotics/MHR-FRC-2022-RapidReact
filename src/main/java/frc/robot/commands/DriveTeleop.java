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
  private double spin;
  private boolean turnInPlace;
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
    forward = -_driverController.getLeftY();
    spin = _driverController.getLeftX();
    //_drivetrain.moveTankDrive(left, right);
    _drivetrain.move(forward, spin);
  }

  // Called once the command en%ds or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.moveTankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
