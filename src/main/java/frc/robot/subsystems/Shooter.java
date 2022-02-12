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
  
  /*private CANSparkMax leftShooterMotor = new CANSparkMax(RobotMap.leftShooterMotor, MotorType.kBrushless);
  private CANSparkMax rightShooterMotor = new CANSparkMax(RobotMap.rightShooterMotor, MotorType.kBrushless);

  private SparkMaxPIDController leftPID;
  private SparkMaxPIDController rightPID;
  
  private double kP = 6e-5;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000015;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;

  public Shooter() {
    SmartDashboard.putNumber("Shooter_speed", .6);
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
    rightShooterMotor.setInverted(true);
  }

  //private TalonSRX aimingMotor = new TalonSRX(RobotMap.armMotor);

  public void setShooter(double rpm) {
    leftPID.setReference(rpm, ControlType.kVelocity); 
    rightPID.setReference(rpm, ControlType.kVelocity); 
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   /* SmartDashboard.putNumber("left rps", leftShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right rps", rightShooterMotor.getEncoder().getVelocity());*/
  }
}
