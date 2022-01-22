// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  public Shooter() {
    SmartDashboard.putNumber("Shooter_speed", .6);
  }
  private Spark leftShooterMotor = new Spark(RobotMap.leftShooterMotor);
  private Spark rightShooterMotor = new Spark(RobotMap.rightShooterMotor);

  //private TalonSRX aimingMotor = new TalonSRX(RobotMap.armMotor);

  public void setShooter(double speed) {
    leftShooterMotor.set(speed); 
    rightShooterMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
