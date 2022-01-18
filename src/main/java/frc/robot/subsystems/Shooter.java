// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  public Shooter() {

  }
  private TalonSRX leftShooterMotor = new TalonSRX(RobotMap.leftShooterMotor);
  private TalonSRX rightShooterMotor = new TalonSRX(RobotMap.rightShooterMotor);

  public TalonSRX getMotor(int motor) {
    if(motor == 0) {
      return leftShooterMotor;
    } else {
      return rightShooterMotor;
    }
  }

  public void setShooter(double speed) {
    leftShooterMotor.set(ControlMode.PercentOutput, speed); //TODO change this to ControlMode.Velocity and use an encoder. 
    rightShooterMotor.set(ControlMode.PercentOutput,-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
