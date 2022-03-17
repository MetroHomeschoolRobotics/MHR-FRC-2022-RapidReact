// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimberWinch extends SubsystemBase {
  /** Creates a new ClimberWinch. */
  public ClimberWinch() {
    climberWinchMotor.setInverted(true);
    climberWinchMotor.burnFlash();
    climberWinchMotor.getEncoder().setPosition(4);
  }

  private CANSparkMax climberWinchMotor = new CANSparkMax(RobotMap.climber_winch, MotorType.kBrushless);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Value", getClimberEncoder());
  }


  public void setClimberWinchMotor(double speed) {
    climberWinchMotor.set(speed);
  }
  public void resetClimberEncoder(double position) {
    climberWinchMotor.getEncoder().setPosition(position);
  }
  public double getClimberEncoder() {
    return climberWinchMotor.getEncoder().getPosition();
  }
}
