// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  Pneumatics sPneumatics;
  public Intake(Pneumatics s_pneumatics) {
    sPneumatics = s_pneumatics;
  }
  //Spark motor controller on pwm to run intake motor
  private VictorSPX intakeMotor = new VictorSPX(RobotMap.intakeMotorPort);
  private VictorSPX Indexer_motor = new VictorSPX(RobotMap.indexerMotor);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(double speed) {//Set intake motor to a percentage output
    if(sPneumatics.getIntake()) {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }
    Indexer_motor.set(VictorSPXControlMode.PercentOutput, speed);
  }


}
