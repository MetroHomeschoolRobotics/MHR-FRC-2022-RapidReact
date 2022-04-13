// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  /** Creates a new DriveDistance. */
  private Drivetrain _drivetrain;
  private double kP = 0.5;
  private double kI = 0;
  private double kD = 0;


  private double _angle;

  private PIDController turnController = new PIDController(.01, 0.01, .0025);

  private double _distance;
  private PIDController pidController = new PIDController(kP, kI, kD);

  public DriveDistance(Drivetrain drivetrain, double distance) {
    _drivetrain = drivetrain;
    _distance = distance;

    addRequirements(_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTolerance(2, 5);
    pidController.setIntegratorRange(-.5, .5);
    _drivetrain.resetEncoders();
    //SmartDashboard.putData(pidController);
    _angle = _drivetrain.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*_drivetrain.moveTank(MathUtil.clamp(pidController.calculate(_drivetrain.getLeftEncoderDistance(), _distance), -0.5, 0.5),
     MathUtil.clamp(pidController.calculate(_drivetrain.getRightEncoderDistance(), _distance), -0.5, 0.5));*/
     _drivetrain.moveManual(MathUtil.clamp(pidController.calculate(.5*(_drivetrain.getLeftEncoderDistance()+_drivetrain.getRightEncoderDistance()), _distance),-.5,.5),
     turnController.calculate(_drivetrain.getHeading(), _angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return pidController.atSetpoint();
  }
}
