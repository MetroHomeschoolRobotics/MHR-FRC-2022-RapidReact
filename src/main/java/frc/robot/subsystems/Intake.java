// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  //Spark motor controller on pwm to run intake motor
  private Spark intakeMotor = new Spark(RobotMap.intakeMotorPort);
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(double speed) {//Set intake motor to a percentage output
    intakeMotor.set(speed);
  }


}
