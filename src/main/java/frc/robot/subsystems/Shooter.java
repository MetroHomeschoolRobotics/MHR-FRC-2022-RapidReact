// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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

  //SparkMax PID controllers
  private SparkMaxPIDController leftPID;
  private SparkMaxPIDController rightPID;
  
  //PID constants taken from SparkMax example
  private double kP = 6e-5; //proportional
  private double kI = 0; //Integral (0 because low torque application)
  private double kD = 0; //Differential (0 because no need to stop as we get close)
  private double kIz = 0; //Integration zone (area where integral is used, is zero because integral is 0)
  private double kFF = 0.000015; //Feed forward (predictive adjustments)
  private double kMaxOutput = 1; //Full speed forward
  private double kMinOutput = -1; //full speed backward

  public Shooter() {
    //Set up PID controllers
    leftPID = leftShooterMotor.getPIDController();
    rightPID = rightShooterMotor.getPIDController();
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIZone(kIz);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);
    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    rightPID.setIZone(kIz);
    rightPID.setFF(kFF);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);
    //Invert Right motor
    rightShooterMotor.setInverted(true);
  }

  //private TalonSRX aimingMotor = new TalonSRX(RobotMap.armMotor);

  public void setShooter(double rps) {
    //Set PID controllers to run to a certain revolutions per second
    leftPID.setReference(rps, ControlType.kVelocity); 
    rightPID.setReference(rps, ControlType.kVelocity); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Print current velocities to the dashboard. 
    SmartDashboard.putNumber("left rps", leftShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right rps", rightShooterMotor.getEncoder().getVelocity());
  }
}
