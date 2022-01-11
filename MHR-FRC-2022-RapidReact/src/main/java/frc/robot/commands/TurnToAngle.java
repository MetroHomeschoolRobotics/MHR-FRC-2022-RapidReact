// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  private Drivetrain _drivetrain;
  private double kP = .03;
  private double kI = .03;
  private double kD = .03;
  private double _setpoint;
  private PIDController pid = new PIDController(kP, kI, kD);
  
  public TurnToAngle(Drivetrain drivetrain, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.\
    _setpoint = setpoint;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.enableContinuousInput(-180.0,180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.moveArcadeDrive(pid.calculate(_drivetrain.getGyro(), _setpoint),0);
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
