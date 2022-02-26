// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  
  //Create shooter SparkMax motor controllers
  private CANSparkMax leftShooterMotor = new CANSparkMax(RobotMap.leftShooterMotor, MotorType.kBrushless);
  private CANSparkMax rightShooterMotor = new CANSparkMax(RobotMap.rightShooterMotor, MotorType.kBrushless);

  private PIDController speedAdjuster = new PIDController(1,0,0);


  public Shooter() {
    SmartDashboard.putData(speedAdjuster);
    leftShooterMotor.setInverted(true);
    leftShooterMotor.burnFlash();
    rightShooterMotor.setInverted(false);
    rightShooterMotor.burnFlash();
  }

  //private TalonSRX aimingMotor = new TalonSRX(RobotMap.armMotor);

  public void setShooterPercentOutput(double speed) {
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void setShooterVelocity(double velocity) {
    leftShooterMotor.set(speedAdjuster.calculate(leftShooterMotor.getEncoder().getVelocity(), 1000));
    rightShooterMotor.set(speedAdjuster.calculate(rightShooterMotor.getEncoder().getVelocity(), 1000));
  }

  public double getAverageVelocity() {
    return (leftShooterMotor.getEncoder().getVelocity()+rightShooterMotor.getEncoder().getVelocity())/2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Print current velocities to the dashboard. 
    SmartDashboard.putNumber("left rps", leftShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right rps", rightShooterMotor.getEncoder().getVelocity());
  }
}
