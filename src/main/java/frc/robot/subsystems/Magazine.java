// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase {
  /** Creates a new Magazine. */
  private VictorSPX Magazine_motor = new VictorSPX(RobotMap.magazineMotor);
  private final AnalogInput ultrasonic = new AnalogInput(1);
  
  public Magazine() {
    
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("ultrasonic value", getUltrasonic());
    //SmartDashboard.putBoolean("Has Ball?", hasBall());
  }
  public void setMagazine(double speed){
    Magazine_motor.set(VictorSPXControlMode.PercentOutput,-speed);
  }

  public double getUltrasonic(){
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    return ultrasonic.getValue() * voltage_scale_factor * 0.0492;
  }

  public boolean hasBall() {
    return getUltrasonic()>3.75||getUltrasonic()>6;
  }
}
