// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase {
  /** Creates a new Magazine. */
  public Magazine() {}
  private VictorSPX Magazine_motor = new VictorSPX(RobotMap.magazineMotor);
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setMagazine(double speed){
    Magazine_motor.set(VictorSPXControlMode.PercentOutput,-speed);
  }
}
