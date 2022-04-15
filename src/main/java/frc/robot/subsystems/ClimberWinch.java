// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimberWinch extends SubsystemBase {
  /** Creates a new ClimberWinch. */
  public ClimberWinch() {
    climberWinchMotor.setInverted(true);
    climberWinchMotor.burnFlash();
    climberWinchMotor.getEncoder().setPosition(4);
    SmartDashboard.putBoolean("Software Limits on Climber", true);
  }

  private static CANSparkMax climberWinchMotor = new CANSparkMax(RobotMap.climber_winch, MotorType.kBrushless);

  private DigitalInput limitSwitch = new DigitalInput(RobotMap.limitSwitchPort);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Value", getClimberEncoder());
    SmartDashboard.putBoolean("Upper Limit Switch", getUpperLimitSwitch());
    SmartDashboard.putBoolean("Lower Limit Switch", getLowerLimitSwitch());
  }

  public boolean getUpperLimitSwitch() {
    return !limitSwitch.get()&&(getClimberEncoder()>=40);
  }

  public boolean getLowerLimitSwitch() {
    return !limitSwitch.get()&&(getClimberEncoder()<40);
  }

  public void setClimberWinchMotor(double speed) {
    if(SmartDashboard.getBoolean("Software Limits on Climber", false)) {
      if(speed<0 && getLowerLimitSwitch()) {
        speed = 0;
      } else if(speed>0 && getUpperLimitSwitch()) {
        speed = 0;
      }
    }
    climberWinchMotor.set(speed);
  }
  public void resetClimberEncoder(double position) {
    climberWinchMotor.getEncoder().setPosition(position);
  }
  public double getClimberEncoder() {
    return climberWinchMotor.getEncoder().getPosition();
  }
  public static double getMotorOutput() {
    return climberWinchMotor.get();
  }
}
