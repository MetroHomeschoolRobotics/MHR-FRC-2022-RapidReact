// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TargetBall extends CommandBase {
  /** Creates a new TargetBall. */
  private Vision _vision;
  private Drivetrain _drivetrain;

  //private PIDController drivePID = new PIDController(0.05, 0, 0);
  private PIDController aimPID = new PIDController(0.015, 0, 0.0002);

  private double turnError;
  private PhotonTrackedTarget currentTarget;
  //private double distanceError;

  public TargetBall(Vision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _vision = vision;
    _drivetrain = drivetrain;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimPID.setTolerance(2, 5);
    SmartDashboard.putData(aimPID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(_vision.intakeHasTarget()){
        currentTarget = _vision.getIntakeTarget();
        turnError = currentTarget.getYaw();
        //distanceError = currentTarget.getPitch();
        if(/*Math.abs(turnError)>2*/true) {
          _drivetrain.moveManual(0.5, -aimPID.calculate(turnError, 0));
        }
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
