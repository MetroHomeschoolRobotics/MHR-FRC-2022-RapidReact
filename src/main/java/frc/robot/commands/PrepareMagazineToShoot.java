// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class PrepareMagazineToShoot extends CommandBase {
  /** Creates a new PrepareMagazineToShoot. */
  Magazine magazine;
  public PrepareMagazineToShoot(Magazine s_magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    magazine=s_magazine;
    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(magazine.getBeamBreak3()) {
      magazine.setMagazine(-.6);
    } else {
      magazine.setMagazine(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !magazine.getBeamBreak3();
  }
}
