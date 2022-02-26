// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class LimelightAim extends CommandBase {
  /** Creates a new LimelightAim. */
  private Drivetrain drivetrain;
  private Arm arm;
  private Vision vision;
  private double TX_threshold;
  private Shooter shooter;

  
  public LimelightAim(Drivetrain _drivetrain, Arm _arm, Vision _vision, Shooter _shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_arm);
    addRequirements(_drivetrain);
    addRequirements(_vision);
    drivetrain = _drivetrain;
    arm = _arm;
    vision = _vision; 
    shooter = _shooter;
  }
  
  private PIDController horizontal_PID = new PIDController(0, 0, 0);
  private PIDController vertical_PID = new PIDController(0, 0, 0);
  private double vertical_threshold = .03;
  private double rps_threshold = 1;
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(vision.getLimelightTX()) > TX_threshold){
      drivetrain.moveManual(0, horizontal_PID.calculate(vision.getLimelightTX(), 0));
    }
    else {
      if(Math.abs(arm.getArmPot()-vision.get_arm_angle(vision.getLimelightTY()))<vertical_threshold) {
        arm.setArmMotor(vertical_PID.calculate(arm.getArmPot(), vision.get_arm_angle(vision.getLimelightTY())));
      }
      //shooter.setShooter(vision.get_shooter_rps(vision.getLimelightTY()));
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(arm.getArmPot()-vision.get_arm_angle(vision.getLimelightTY()))<vertical_threshold && Math.abs(shooter.getAverageVelocity()-vision.get_shooter_rps(vision.getLimelightTY()))<rps_threshold) {
      return true;
    } else {
      return false;
    }
  }
}
