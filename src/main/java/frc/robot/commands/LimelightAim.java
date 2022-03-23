// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  private boolean turn;
  private int count = 0;

  private XboxController controller;
  
  public LimelightAim(Drivetrain _drivetrain, Vision _vision, XboxController _controller, boolean _turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drivetrain);
    addRequirements(_vision);
    drivetrain = _drivetrain;
    controller = _controller;
turn = _turn;
    vision = _vision; 

  }
  
  private static PIDController horizontal_PID = new PIDController(0.03, 0, 0.001);
  private double horizontal_Threshold  = .12;
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("shooter RPM given angle", vision.get_shooter_rps(vision.getLimelightTY()));
controller.setRumble(RumbleType.kLeftRumble, .2);
vision.setPIP(1);
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
    controller.setRumble(RumbleType.kLeftRumble, .0);
    //CommandScheduler.getInstance().schedule(new AngleArm(vision.get_arm_angle(vision.getLimelightTY()), RobotContainer.s_arm));
    vision.setPIP(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(vision.getLimelightHasTarget()) {
      if(Math.abs(vision.getLimelightTX())<horizontal_Threshold) {
        count+=1;
        SmartDashboard.putNumber("shooter RPM given angle", vision.get_shooter_rps(vision.getLimelightTY()));
        SmartDashboard.putNumber("arm from limelight", vision.get_arm_angle(vision.getLimelightTY()));
        return true;
      }
      } else {
        count = 0;
      }
      if(count>0) {
        return true;
      } else {
        return false;
      }
  }
}
