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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.kauailabs.navx.frc.AHRS;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  
  //create sparkmax motor controllers
  private static CANSparkMax frontLeft = new CANSparkMax(RobotMap.leftFrontMotor, MotorType.kBrushless);
  private static CANSparkMax rearLeft = new CANSparkMax(RobotMap.leftRearMotor, MotorType.kBrushless);
  private static CANSparkMax frontRight = new CANSparkMax(RobotMap.rightFrontMotor, MotorType.kBrushless);
  private static CANSparkMax rearRight = new CANSparkMax(RobotMap.rightRearMotor, MotorType.kBrushless);


  //private ADIS16448_IMU gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  //using this constructor to prevent a known WPILib Issue
  //see https://docs.wpilib.org/en/latest/docs/yearly-overview/known-issues.html#adis16448-not-reading-values-in-java
  
  //We may switch back to navx at some point in the future
  //private AHRS gyro = new AHRS(Port.kUSB1); //constructor for USB navx
  //private AHRS gyro = new AHRS(SPI.Port.kMXP); //constructor for MXP navx

  //Create field2d widget to view robot pose and trajectory in glass
  private final Field2d m_field = new Field2d();
  

  //variables to convert encoder distances to meters.  
  private static final double kGearRatio = 10.71;//gear ratio, in inches
  private static final double kWheelRadiusInches = 3.0; //radius of wheels, in inches  

  //Odometry variable for trajectory following; stores the robot's current position
  private final DifferentialDriveOdometry m_odometry;

  double currentGyroZero = 0;

  //DifferentialDrive object to help manage motors
  private DifferentialDrive differentialDrivetrain = new DifferentialDrive(frontLeft, frontRight);
  

  public Drivetrain() {
    SmartDashboard.putData("field", m_field);
    differentialDrivetrain.setMaxOutput(1);
    differentialDrivetrain.setDeadband(.01);
    gyro.reset();
    //Right side must spin in reverse for robot to drive forward
    frontRight.setInverted(true);
    rearRight.setInverted(true);
    //Rear motors have the same output as front motors because they are in the same gearboxes
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    //Convert drivetrain motor revolutions to distances of robot travel (in meters)
    frontLeft.getEncoder().setPositionConversionFactor((Units.inchesToMeters(kWheelRadiusInches)*2*Math.PI)/(kGearRatio));
    frontRight.getEncoder().setPositionConversionFactor((Units.inchesToMeters(kWheelRadiusInches)*2*Math.PI)/(kGearRatio));
    //Set the encoders to zero when the robot starts
    resetEncoders();
    //Initialize odometry
    m_odometry = new DifferentialDriveOdometry(getRotation2d());
  }

  @Override
  public void periodic() {// This method will be called once per scheduler run
  //Print Gyro compass to the dashboard  
	SmartDashboard.putData(gyro);
  SmartDashboard.putNumber("gyro angle", getHeading());
  //Print encoder distances to the dashboard
  SmartDashboard.putNumber("Front Left Encoder", getLeftEncoderDistance());
  SmartDashboard.putNumber("Front Right Encoder", getRightEncoderDistance());
  //Update current robot pose
  m_odometry.update(getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  //Show current pose on the field
  m_field.setRobotPose(getPose());
  }

  public Field2d getField2d() {
    //allow getting the Field widget from other classes to put the trajectory on it
    return m_field;
  }

  public Pose2d getPose() {
    //get the robot's current position
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //get the current wheel speeds for trajectory work
    return new DifferentialDriveWheelSpeeds(frontLeft.getEncoder().getVelocity(), frontRight.getEncoder().getVelocity());
  }



  public void move(double forward, double spin) {
    //Teleop moving command; squares outputs to the motor
    differentialDrivetrain.arcadeDrive(forward, spin, true);
  }

  public void moveManual(double forward, double spin) {
    //Arcade drive command without deadzone or squared outputs
    frontLeft.set(forward+spin);
    frontRight.set(forward-spin);
    differentialDrivetrain.feed();
  }
  public void moveTank(double left, double right) {
    //Set each motor output directly
    frontLeft.set(left);
    frontRight.set(right);
    differentialDrivetrain.feed();
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //Set each motor's voltage; useful for physically meaningful outputs
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    differentialDrivetrain.feed();
  }
  
  public void resetOdometry(Pose2d pose) {
    //Reset the robot odometry to say we are at a specific spot
    //used at the start of auto and other trajectories 
    resetEncoders();
    m_odometry.resetPosition(pose, getRotation2d());
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

  public RelativeEncoder getLeftEncoder() {//get the left encoder object
    return frontLeft.getEncoder();
  }

  public RelativeEncoder getRightEncoder() {//get the right encoder object
    return frontRight.getEncoder();
  }

  public double getAverageEncoderDistance() {//average distance the robot has traveled
    return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
  }

  public double getTurnRate() {//rate of change of heading
    return -gyro.getRate();
  }

  public double getHeading() {//current heading
    return gyro.getAngle();
  }
  public Rotation2d getRotation2d() {//current heading in trajectory following format
    return Rotation2d.fromDegrees(getHeading());
  }
  public void zeroHeading() {//reset gyro heading to 0
    gyro.reset();
  }


  public void resetEncoders() {//reset both encoders to the zero position. 
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
  }
}