// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinArmAtSDRPM extends CommandBase {
  /** Creates a new SpinShooter. */
  private Shooter _shooter;
  boolean sd = false;
  public SpinArmAtSDRPM(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    addRequirements(_shooter);
    SmartDashboard.putNumber("Manual RPM", 3500);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.setShooterVelocity(SmartDashboard.getNumber("Manual RPM", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.setShooterPercentOutput(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
