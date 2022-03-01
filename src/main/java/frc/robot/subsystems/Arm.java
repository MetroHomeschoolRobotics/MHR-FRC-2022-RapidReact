// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //String potentiometer for measuring arm position
  private AnalogPotentiometer armPot = new AnalogPotentiometer(0);
  private TalonSRX arm_motor = new TalonSRX(RobotMap.arm_winch);
  
  private double maxPotOutput = 0.8;
  private double minPotOutput = 0.1;
  
  public Arm() {
  
  }


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

  public void setArmMotor(double speed) {
    if (speed > 0 && getArmPot() >= maxPotOutput){
      //speed = 0;
    }
    else if ( speed < 0 && getArmPot() <= minPotOutput){
      //speed = 0;
    }
    arm_motor.set(ControlMode.PercentOutput, speed);
  }

}
