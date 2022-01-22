// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
  

  //Drive constraints for auto
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  public static final double kTrackWidthMeters = .69;
  
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  
  //use CANSparkMax.getEncoder() to get the RelativeEncoder type. 

  //variables to convert encoder distances to meters.  
  private static final double kGearRatio = 10.71;//gear ratio, in inches
  private static final double kWheelRadiusInches = 3.0; //radius of wheels, in inches
  private static final double kTicksPerRevolution = 42;//number of ticks per revolution of the encoder. For neo motor encoder, this is 42. 

  

  AHRS gyro = new AHRS(SPI.Port.kMXP);


  //variables for trajectory work; will be used only in autonomous
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);
  Pose2d pose = new Pose2d();

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
	odometry.update(navx.getRotation2d(), ticksToMeters(frontLeft.getEncoder().getPosition()), ticksToMeters(frontRight.getEncoder().getPosition()));

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
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        frontLeft.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        frontRight.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }
  public void zeroHeading() {
    navx.reset();
  }
  public double getTurnRate() {
    return -navx.getRate();
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public void tankDriveVoltage(double left, double right) {//Control each side's motors individually. 
  frontLeft.setVoltage(left);
  frontRight.setVoltage(right);
  differentialDrivetrain.feed();
  }
  public double getAverageEncoderDistance() {
    return (frontLeft.getEncoder().getPosition() + frontRight.getEncoder().getPosition()) / 2.0;
  }

  
//TODO: make encoder speed functions and a function to get Odometry

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