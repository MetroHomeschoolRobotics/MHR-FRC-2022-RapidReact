// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AimDrivetrain extends CommandBase {
  /** Creates a new AimDrivetrain. */
  private Drivetrain _drivetrain;
  private Vision _vision;
  private double distanceError;
  private double turnError;

  private PIDController drivePID = new PIDController(0.1, 0, 0.001);
  private PIDController aimPID = new PIDController(0.02, 0, 0.002);
  public AimDrivetrain(Vision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(drivetrain);
    _vision = vision;
    _drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putData(aimPID);
    SmartDashboard.putData(drivePID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_vision.limelightHasTarget()) {
      PhotonTrackedTarget target = _vision.getLimelightTarget();
      turnError = target.getYaw();
      distanceError = target.getPitch();
      _drivetrain.moveManual(-drivePID.calculate(distanceError,0),-aimPID.calculate(turnError, 0));
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
