// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private Intake _intake;
  /** Creates a new RunIntake. */
  public RunIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Intake direction", "forward");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.setIntake(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.setIntake(0);
    SmartDashboard.putString("Intake direction", "stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
