// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;


public class LimelightAim extends CommandBase {
  /** Creates a new LimelightAim. */
  private Drivetrain drivetrain;
  private Vision vision;
  private double TX_threshold;

  
  public LimelightAim(Drivetrain _drivetrain, Vision _vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drivetrain);
    addRequirements(_vision);
    drivetrain = _drivetrain;

    vision = _vision; 

  }
  
  private PIDController horizontal_PID = new PIDController(0.015, 0, 0.0003);
  private double horizontal_Threshold  = .1;
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("shooter RPM given angle", vision.get_shooter_rps(vision.getLimelightTY()));
SmartDashboard.putData(horizontal_PID);
CommandScheduler.getInstance().schedule(new AngleArm(vision.get_arm_angle(vision.getLimelightTY()), RobotContainer.s_arm));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("arm from limelight", vision.get_arm_angle(vision.getLimelightTY()));
    if (Math.abs(vision.getLimelightTX()) > TX_threshold){
      drivetrain.moveManual(0, -horizontal_PID.calculate(vision.getLimelightTX(), 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(vision.getLimelightTX())<horizontal_Threshold) {
      return true;
    } else {
      return false;
    }
  }
}
