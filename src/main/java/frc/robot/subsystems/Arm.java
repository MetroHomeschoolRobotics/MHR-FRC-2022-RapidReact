// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //String potentiometer for measuring arm position
  private AnalogPotentiometer armPot = new AnalogPotentiometer(0);
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Print armpot value to the dashboard every cycle
    SmartDashboard.putNumber("Arm Potentiometer Value", armPot.get());
  }
  //Get current armpot value
  public double getArmPot() {
      return armPot.get();
  }
}
