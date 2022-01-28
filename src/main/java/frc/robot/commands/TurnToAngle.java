// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  //these settings work best to get closest to the target angle
  private double kP = .017;
  private double kI = 0;
  private double kD = 0.0025;

  private double _angle;
  private Drivetrain _drivetrain;

  private PIDController turnController = new PIDController(kP, kI, kD);

  

  public TurnToAngle(double angle, Drivetrain drivetrain) {
    _angle = angle;
    //_angle = SmartDashboard.getNumber("desired angle", 90);
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(3, 5);
    turnController.setIntegratorRange(-.5, .5);
    //if(!(Math.abs(_drivetrain.getHeading()-_angle)<45)){
      //turnController.setI(0);
    //}
    SmartDashboard.putData(turnController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.moveManual(0, turnController.calculate(_drivetrain.getHeading(),_angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.move(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return turnController.atSetpoint();
  }
}
