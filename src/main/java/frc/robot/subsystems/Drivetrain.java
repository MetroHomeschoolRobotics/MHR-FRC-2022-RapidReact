// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
  
  
  private AHRS navx = new AHRS(Port.kUSB1);


  private final Field2d m_field = new Field2d();
  
  
  //use CANSparkMax.getEncoder() to get the RelativeEncoder type. 

  //variables to convert encoder distances to meters.  
  private static final double kGearRatio = 10.71;//gear ratio, in inches
  private static final double kWheelRadiusInches = 3.0; //radius of wheels, in inches  

  private final DifferentialDriveOdometry m_odometry;

  AHRS gyro = new AHRS(SPI.Port.kMXP);



  private DifferentialDrive differentialDrivetrain = new DifferentialDrive(frontLeft, frontRight);
  

  public Drivetrain() {
    SmartDashboard.putData("field", m_field);
  differentialDrivetrain.setMaxOutput(1);
  differentialDrivetrain.setDeadband(.01);
	navx.reset();
  frontRight.setInverted(true);
  rearRight.setInverted(true);
  rearLeft.follow(frontLeft);
  rearRight.follow(frontRight);
  System.out.println(frontLeft.getEncoder().getPositionConversionFactor());
  frontLeft.getEncoder().setPositionConversionFactor((Units.inchesToMeters(kWheelRadiusInches)*2*Math.PI)/(kGearRatio));
  //frontLeft.getEncoder().setVelocityConversionFactor(frontLeft.getEncoder().getPositionConversionFactor());
  frontRight.getEncoder().setPositionConversionFactor((Units.inchesToMeters(kWheelRadiusInches)*2*Math.PI)/(kGearRatio));
  //frontRight.getEncoder().setVelocityConversionFactor(frontRight.getEncoder().getPositionConversionFactor());
  resetEncoders();
  m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
	SmartDashboard.putNumber("Gyro", navx.getAngle());
  SmartDashboard.putData(navx);
  SmartDashboard.putNumber("Front Left Encoder", frontLeft.getEncoder().getPosition());
  SmartDashboard.putNumber("Front Right Encoder", frontRight.getEncoder().getPosition());
  m_odometry.update(gyro.getRotation2d(), frontLeft.getEncoder().getPosition(), frontRight.getEncoder().getPosition());
  m_field.setRobotPose(getPose());
  }

  public Field2d getField2d() {
    return m_field;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeft.getEncoder().getVelocity(), frontRight.getEncoder().getVelocity());
  }



  public void move(double forward, double spin) {
      differentialDrivetrain.arcadeDrive(forward, spin,true);
  }

  public void moveManual(double forward, double spin) {
    frontLeft.set(forward+spin);
    frontRight.set(forward-spin);
    differentialDrivetrain.feed();
  }
  public void moveTank(double left, double right) {
    frontLeft.set(left);
    frontRight.set(right);
    differentialDrivetrain.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    differentialDrivetrain.feed();
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
  public double encoderToInches(double encoder) {
    encoder/=kGearRatio;//Now we have revolutions of the drive wheels
    encoder/=kWheelRadiusInches*2*Math.PI; //Now we have inches
    return encoder;
  }

  public RelativeEncoder getLeftEncoder() {
    return frontLeft.getEncoder();
  }

  public RelativeEncoder getRightEncoder() {
    return frontRight.getEncoder();
  }

  public double getAverageEncoderDistance() {
    return (frontLeft.getEncoder().getPosition() + frontRight.getEncoder().getPosition())/2;
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public double getHeading() {
    return navx.getAngle();
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