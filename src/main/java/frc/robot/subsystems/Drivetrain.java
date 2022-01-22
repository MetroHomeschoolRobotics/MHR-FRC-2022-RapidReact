// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
  private MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeft, rearLeft);
  private MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRight, rearRight);
  
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double ticksToDistanceFactor = 1/(42*10.71*6*Math.PI);//Neo motor encoders count 42 ticks per revolution. Every 10.71 revolutions (the gear ratio of the chassis) the wheels go around once. Each time the wheels go around the robot travels the circumference of the 6 in wheel. 
  //use CANSparkMax.getEncoder() to get the RelativeEncoder type. 

  
  private static final double kGearRatio = 10.71;//gear ratio, in inches
  private static final double kWheelRadiusInches = 3.0; //radius of wheels, in inches
  private static final double kDrivetrainWidth = 0;//TODO measure this distance between centers of wheels, in inches. 
  private static final double kTicksPerRevolution = 42;//number of ticks per revolution of the encoder. For neo motor encoder, this is 42. 

  

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kDrivetrainWidth));
  private DifferentialDrive differentialDrivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);


  public Drivetrain() {
	navx.reset();
  rightMotorGroup.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	SmartDashboard.putNumber("Gyro", -navx.getAngle());
	
  }
  public void move(double forward, double spin) {
    if(forward<.3) {
      differentialDrivetrain.arcadeDrive(forward/2, spin/2);
    } else {
      differentialDrivetrain.curvatureDrive(forward/2, spin/2,false);
    }
  }
  
  public void moveTankDrive(double left, double right) {//Control each side's motors individually. 
    //differentialDrivetrain.tankDrive(left,right,true);
    leftMotorGroup.set(-left/2);
    rightMotorGroup.set(right/2);
  }

  public double getLeftEncoderDistance() {//get how far the wheels on the LEFT side of the robot have travelled. (Units are in inches)
    return frontLeft.getEncoder().getPosition()*ticksToDistanceFactor;
  }

  public double getRightEncoderDistance() {//get how far the wheels on the RIGHT side of the robot have travelled. (Units are in inches)
    return frontRight.getEncoder().getPosition()*ticksToDistanceFactor;
  }
  
//TODO: make encoder speed functions and a function to get Odometry

  public void resetEncoders() {//reset both encoders to the zero position. 
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
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