// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  
  //Create shooter SparkMax motor controllers
  private CANSparkMax leftShooterMotor = new CANSparkMax(RobotMap.leftShooterMotor, MotorType.kBrushless);
  private CANSparkMax rightShooterMotor = new CANSparkMax(RobotMap.rightShooterMotor, MotorType.kBrushless);

  //private PIDController speedAdjuster = new PIDController(1,0,0);
  
   private SparkMaxPIDController leftshooter_PID;
   private SparkMaxPIDController rightshooter_PID;
   private double kp = 0.000;
   private double kff = 0.00017;
   //private double max_RPM = 5700;
   
  
  
  public Shooter() {
    leftshooter_PID = leftShooterMotor.getPIDController();
    rightshooter_PID = rightShooterMotor.getPIDController();
    leftshooter_PID.setP(kp);
    leftshooter_PID.setFF(kff);
    rightshooter_PID.setP(kp);
    rightshooter_PID.setFF(kff);
    leftShooterMotor.setInverted(true);
    leftShooterMotor.burnFlash();
    rightShooterMotor.setInverted(false);
    rightShooterMotor.burnFlash();
    leftshooter_PID.setOutputRange(0, 1);
    rightshooter_PID.setOutputRange(0, 1);
  }

  public void setShooterPercentOutput(double speed) {
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void setShooterVelocity(double velocity) {
    leftshooter_PID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    rightshooter_PID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
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
