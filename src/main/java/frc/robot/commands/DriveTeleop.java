// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTeleop extends CommandBase {
  /** Creates a new DriveTeleop. */
  private Drivetrain _drivetrain;
  private XboxController _driverController;
  private double forward;
  private double spin;
  private double kP = .017;
  private double kI = 0;
  private double kD = 0.0025;
  private boolean lastButtonValue = false;
  private PIDController turnController = new PIDController(kP, kI, kD);
  private double measurement = 0;
  //private SlewRateLimiter filter = new SlewRateLimiter(0.5);
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
    
    if(_driverController.getLeftStickButton()) {
      if(!lastButtonValue) {
        measurement = _drivetrain.getHeading();
      }
      spin = turnController.calculate(_drivetrain.getHeading(), measurement);
    }
    lastButtonValue = _driverController.getLeftStickButton();
    
    if(_driverController.getLeftTriggerAxis()>.2) {
      _drivetrain.setMaxOutput(.5+(_driverController.getLeftTriggerAxis()/2));
      _drivetrain.MoveCurvature(forward/2, spin);
    } else if(_driverController.getRightTriggerAxis()>.2) {
      _drivetrain.setMaxOutput(.25);
      _drivetrain.move(forward, spin, true);
    }
     else {
      _drivetrain.setMaxOutput(.5);
      _drivetrain.move(forward, spin, true);
    }
    SmartDashboard.putNumber("forward", forward);
    // _drivetrain.moveTank(_driverController.getLeftY(), _driverController.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.move(0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
