// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import frc.robot.RobotMap;

public class Magazine extends SubsystemBase {
  /** Creates a new Magazine. */
  private VictorSPX Magazine_motor = new VictorSPX(RobotMap.magazineMotor);
  private final AnalogInput ultrasonic = new AnalogInput(1);
  private final PicoColorSensor colorSensor = new PicoColorSensor();
  private SendableChooser<Integer> colorChooser = new SendableChooser<Integer>();
  private frc.robot.PicoColorSensor.RawColor color;
  private DigitalInput beamBreak1 = new DigitalInput(2);
  private DigitalInput beamBreak2 = new DigitalInput(1);
  private DigitalInput beamBreak3 = new DigitalInput(3);
  public Magazine() {
    colorChooser.setDefaultOption("blue", 1);
    colorChooser.addOption("red",2);
    colorChooser.addOption("none", 0);
    SmartDashboard.putData(colorChooser);
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("ultrasonic value", getUltrasonic());
    //SmartDashboard.putBoolean("Has Ball?", hasBall());
    color = colorSensor.getRawColor0();
    SmartDashboard.putNumber("Red", color.red);
    SmartDashboard.putNumber("Blue", color.blue);
    SmartDashboard.putNumber("Green", color.green);
    SmartDashboard.putNumber("IR", color.ir);
    SmartDashboard.putNumber("Distance", colorSensor.getProximity0());
    SmartDashboard.putBoolean("beam1", getBeamBreak1());
    SmartDashboard.putBoolean("beam2", getBeamBreak2());
    SmartDashboard.putBoolean("beam3", getBeamBreak3());
    
    if(colorSensor.getProximity0()>1000) {
      if(color.red>color.green) {
        SmartDashboard.putString("Color", "red");
      } else if(color.green>color.blue) {
        SmartDashboard.putString("Color", "blue");
      }
    } else {
      SmartDashboard.putString("Color", "none");
    }

  }
  public void setMagazine(double speed){
    Magazine_motor.set(VictorSPXControlMode.PercentOutput,-speed);
  }

  public boolean colorSensorHasBall() {
    return isRightColor()!=2;
  }

  public int isRightColor() {//0 is no, 1 is yes, 2 is no ball exists
    if(colorChooser.getSelected()==0) {
      return 1;
    } else if(colorSensor.getProximity0()>800) {
      if(colorChooser.getSelected()==1&& color.green>color.blue) {
        return 1;
      } else if(colorChooser.getSelected()==2 && color.red>color.green){
        return 1;
      } else {
        return 0;
      }
    } else {
      return 2;
    }
  }
  public boolean getBeamBreak1() {
    return !beamBreak1.get();
  }
  public boolean getBeamBreak2() {
    return !beamBreak2.get();
  }
  public boolean getBeamBreak3() {
    return !beamBreak3.get();
  }
}
