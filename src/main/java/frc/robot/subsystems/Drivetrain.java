// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  
  //create sparkmax motor controllers
  private CANSparkMax frontLeft = new CANSparkMax(RobotMap.leftFrontMotor, MotorType.kBrushless);
  private CANSparkMax rearLeft = new CANSparkMax(RobotMap.leftRearMotor, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(RobotMap.rightFrontMotor, MotorType.kBrushless);
  private CANSparkMax rearRight = new CANSparkMax(RobotMap.rightRearMotor, MotorType.kBrushless);
  //group each of the left and right motor controllers together so their inputs are the same
  //private MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeft, rearLeft);
  //private MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRight, rearRight);
  
  
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  
  //use CANSparkMax.getEncoder() to get the RelativeEncoder type. 

  //variables to convert encoder distances to meters.  
  private static final double kGearRatio = 10.71;//gear ratio, in inches
  private static final double kWheelRadiusInches = 3.0; //radius of wheels, in inches
  private static final double kTicksPerRevolution = 42;//number of ticks per revolution of the encoder. For neo motor encoder, this is 42. 

  

  AHRS gyro = new AHRS(SPI.Port.kMXP);



  private DifferentialDrive differentialDrivetrain = new DifferentialDrive(frontLeft, frontRight);
  

  public Drivetrain() {
  differentialDrivetrain.setMaxOutput(1);
  differentialDrivetrain.setDeadband(.01);
	navx.reset();
  frontRight.setInverted(true);
  rearRight.setInverted(true);
  rearLeft.follow(frontLeft);
  rearRight.follow(frontRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putData(differentialDrivetrain);
	SmartDashboard.putNumber("Gyro", -navx.getAngle());
  SmartDashboard.putData(navx);
  }

  public void move(double forward, double spin) {
      differentialDrivetrain.arcadeDrive(forward, spin,true);
  }
  


  public double getLeftEncoderDistance() {//get how far the wheels on the LEFT side of the robot have travelled. 
    return frontLeft.getEncoder().getPosition();
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrivetrain.setMaxOutput(maxOutput);
  }

  public double getRightEncoderDistance() {//get how far the wheels on the RIGHT side of the robot have travelled. 
    return frontRight.getEncoder().getPosition();
  }

  //Trajectory Methods
  public double ticksToMeters(double ticks) {
    ticks/=kTicksPerRevolution; //now we have revolutions of the motor shaft
    ticks/=kGearRatio;//Now we have revolutions of the drive wheels
    ticks*=Units.inchesToMeters(kWheelRadiusInches)*2*Math.PI; //Now we have meters
    return ticks;
  }
  public double getHeading() {
    return -navx.getAngle();
  }
  public void zeroHeading() {
    navx.reset();
  }


  public void resetEncoders() {//reset both encoders to the zero position. 
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
  }

  

  public CANSparkMax getMotor(int position) {
    if(position==0) {
      return frontLeft;
    } else if(position==1) {
      return rearLeft;
    } else if(position==2) {
      return frontRight;
    } else {
      return rearRight;
    }
  }
}