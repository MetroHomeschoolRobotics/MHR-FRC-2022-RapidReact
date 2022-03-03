// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimberWinch extends SubsystemBase {
  /** Creates a new ClimberWinch. */
  public ClimberWinch() {}

  private TalonSRX climberWinchMotor = new TalonSRX(RobotMap.climber_winch);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setClimberWinchMotor(double speed) {
    climberWinchMotor.set(ControlMode.PercentOutput, speed);
  }
}
