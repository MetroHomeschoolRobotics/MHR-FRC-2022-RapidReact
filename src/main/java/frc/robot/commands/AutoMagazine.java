// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoMagazine extends CommandBase {
  /** Creates a new AutoMagazine. */
  private Magazine magazine;
  private Intake intake;
  private Shooter shooter;
  private XboxController controller;
  private int currentColorBall = 1;
  private int lastColorBall=1;
  private boolean hasTwoBalls = false;
  private double indexerSpeed = .4;
  

  public AutoMagazine(Magazine s_magazine, Intake s_intake, Shooter s_shooter, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    magazine=s_magazine;
    intake=s_intake;
    shooter = s_shooter;
    controller = driverController;
    addRequirements(s_intake, s_magazine, s_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRightBumper()) {
      intake.setIntake(1);
      
    } else {
      //intake.setIndexer(0);
    }
    intake.setIndexer(.3);
    if(magazine.colorSensorHasBall()) {
        currentColorBall = magazine.isRightColor();
        if(magazine.getBeamBreak1()||magazine.getBeamBreak3()) {
          hasTwoBalls = true;
        } else {
          hasTwoBalls = false;
        }
        if(currentColorBall==0) {
          controller.setRumble(RumbleType.kLeftRumble, .3);
        }
    } else {
      controller.setRumble(RumbleType.kLeftRumble, 0);
    }
    SmartDashboard.putNumber("Current color", currentColorBall);
    SmartDashboard.putNumber("Last color", currentColorBall);

    if(currentColorBall == 1) {
      //right color
      indexerSpeed = .4;
      shooter.setShooterVelocity(0);
      if(!(magazine.getBeamBreak1()||magazine.getBeamBreak3())||magazine.getBeamBreak2()) {
        magazine.setMagazine(.4);
      } else {
        magazine.setMagazine(0);

      }
    } else if(currentColorBall == 0) {
      if(magazine.getBeamBreak1()&&magazine.getBeamBreak2()) {
          magazine.setMagazine(-.075);
          shooter.setShooterVelocity(0);
        intake.setIndexer(1);
      } else if(hasTwoBalls&&!magazine.getBeamBreak1() && magazine.getBeamBreak2()&&magazine.getBeamBreak3()) {
        //magazine.setMagazine(-.6);
      }else if(!hasTwoBalls){
        magazine.setMagazine(.4);
        shooter.setShooterVelocity(1000);
      } else {
        shooter.setShooterVelocity(0);
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