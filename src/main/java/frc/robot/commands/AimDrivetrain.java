// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AimDrivetrain extends CommandBase {
  /** Creates a new AimDrivetrain. */
  private Drivetrain _drivetrain;
  private Vision _vision;
  private double distanceError;
  private double turnError;
  public AimDrivetrain(Vision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(drivetrain);
    _vision = vision;
    _drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_vision.limelightHasTarget()) {
      PhotonTrackedTarget target = _vision.getLimelightTarget();
      turnError = target.getYaw();
      distanceError = target.getPitch();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
