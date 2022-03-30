// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinShooter extends CommandBase {
  /** Creates a new SpinShooter. */
  private Shooter _shooter;
  private XboxController _driverController;
  private Vision vision;
  private double RPM=0;
  boolean sd = false;
  public SpinShooter(Shooter shooter, XboxController driverController, double _RPM, Vision s_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    vision = s_vision;
    RPM = 0;
    RPM = _RPM;
    _driverController = driverController;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    _driverController.setRumble(RumbleType.kRightRumble, .2);
    //if(RPM == 0) {
      //sd = true;
      //RPM = SmartDashboard.getNumber("shooter RPM given angle",0);
    //}
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RPM!=0) {
    _shooter.setShooterVelocity(RPM);
    } else {
      _shooter.setShooterVelocity(vision.get_shooter_rps(vision.getLimelightTY()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.setShooterPercentOutput(0);
    _driverController.setRumble(RumbleType.kRightRumble, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
