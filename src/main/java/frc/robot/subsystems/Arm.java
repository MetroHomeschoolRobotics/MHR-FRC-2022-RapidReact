// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //String potentiometer for measuring arm position
  private AnalogPotentiometer armPot = new AnalogPotentiometer(0);
  private TalonSRX arm_motor = new TalonSRX(RobotMap.arm_winch);
  private PIDController armPID = new PIDController(35, 0, 0);
  public static final double maxPotOutput = 0.4;
  public static final double minPotOutput = 0.047;
  public double potOutputToHold = .17;
  public boolean hold = true;
  
  public Arm() {
    arm_motor.setInverted(true);
    arm_motor.setNeutralMode(NeutralMode.Brake);
    //SmartDashboard.putData(armPID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Print armpot value to the dashboard every cycle
    //SmartDashboard.putNumber("arm from limelight rc", RobotContainer.armFromLimelight);
    if(hold) {
      if(true) {
      arm_motor.set(ControlMode.PercentOutput, armPID.calculate(getArmPot(), potOutputToHold));
      }
    }
    SmartDashboard.putNumber("Arm Potentiometer Value", armPot.get());
  }
  //Get current armpot value
  public double getArmPot() {
      return armPot.get();
  }

  public void setArmMotor(double speed) {
    if (speed > 0 && getArmPot() >= maxPotOutput){
      speed = 0;
    }
    else if ( speed < 0 && getArmPot() <= minPotOutput){
      speed = 0;
    }
    if(Math.abs(speed)<.1) {
      if(!hold) {
        potOutputToHold = getArmPot();
      }
      hold = true;
    } else {
      hold = false;
    }
    arm_motor.set(ControlMode.PercentOutput, speed);
  }

}
