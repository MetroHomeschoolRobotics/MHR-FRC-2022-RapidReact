// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;


public class LimelightAim extends CommandBase {
  /** Creates a new LimelightAim. */
  private Drivetrain drivetrain;
  private Vision vision;

  private XboxController controller;
  
  public LimelightAim(Drivetrain _drivetrain, Vision _vision, XboxController _controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drivetrain);
    addRequirements(_vision);
    drivetrain = _drivetrain;
    controller = _controller;
    vision = _vision; 

  }
  
  private static PIDController horizontal_PID = new PIDController(0.03, 0.0, 0.002);
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //SmartDashboard.putNumber("shooter RPM given angle", vision.get_shooter_rps(vision.getLimelightTY()));
    controller.setRumble(RumbleType.kLeftRumble, .2);
    horizontal_PID.calculate(vision.getLimelightTX(), 0);
    horizontal_PID.setTolerance(1,10);
    horizontal_PID.setIntegratorRange(-.1, .1);
    
    
    vision.setPIP(1);
    SmartDashboard.putData(horizontal_PID);
//CommandScheduler.getInstance().schedule(new AngleArm(vision.get_arm_angle(vision.getLimelightTY()), RobotContainer.s_arm));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("arm from limelight", vision.get_arm_angle(vision.getLimelightTY()));
    drivetrain.moveManual(0, -horizontal_PID.calculate(vision.getLimelightTX(), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kLeftRumble, .0);
    //CommandScheduler.getInstance().schedule(new AngleArm(vision.get_arm_angle(vision.getLimelightTY()), RobotContainer.s_arm));
    vision.setPIP(2);
    drivetrain.move(0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return horizontal_PID.atSetpoint();
  }
}
