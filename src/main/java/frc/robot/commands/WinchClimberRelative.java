// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberWinch;

public class WinchClimberRelative extends CommandBase {
  /** Creates a new WinchToSetpoint. */
  private PIDController winchPID = new PIDController(0.05, 0, 0);
  private ClimberWinch winch;
  private double setpoint;
  private double amount;
  public WinchClimberRelative(double _amount, ClimberWinch _Winch) {
    // Use addRequirements() here to declare subsystem dependencies.
    winch = _Winch;
    amount = _amount;
    addRequirements(winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    winchPID.setTolerance(3);
    setpoint = winch.getClimberEncoder()+amount;
    //SmartDashboard.putData(winchPID);
    winchPID.calculate(winch.getClimberEncoder(), setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    winch.setClimberWinchMotor(winchPID.calculate(winch.getClimberEncoder(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return winchPID.atSetpoint();
  }
}
