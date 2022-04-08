// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,RobotMap.intakeSolenoid1, RobotMap.intakeSolenoid2);
  private DoubleSolenoid hookSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.hookSolenoid1, RobotMap.hookSolenoid2);
  private DoubleSolenoid hookSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.hookSolenoid3, RobotMap.hookSolenoid4);
  
  public Pneumatics() {
    //setCompressor(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Compressor Running", getCompressorState());
    SmartDashboard.putBoolean("Pressure Switch", getSwitch());
    //hookSolenoid2.set(hookSolenoid1.get());
  }
  
  public void setIntake(boolean down) {
    if(down) {
      intakeSolenoid.set(Value.kForward);
    } else {
      intakeSolenoid.set(Value.kReverse);
    }
  }

  public boolean getIntake() {
    if(intakeSolenoid.get()==Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public void setHook(boolean down) {
    if(down) {
      hookSolenoid1.set(Value.kForward);
      hookSolenoid2.set(Value.kForward);
    } else {
      hookSolenoid1.set(Value.kReverse);
      hookSolenoid2.set(Value.kReverse);
    }
  }

  public boolean getHook() {
    if(hookSolenoid1.get()==Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public void setHook(Value direction) {
    hookSolenoid1.set(direction);
    hookSolenoid2.set(direction);
  }

  public void toggleHooks() {
    hookSolenoid1.toggle();
    hookSolenoid2.toggle();
  }

  public boolean getSwitch() {
    return compressor.getPressureSwitchValue();
  }

  public boolean getCompressorState() {
    return compressor.enabled();//get if the compressor is on or off
  }

  public void setCompressor(boolean on){//turn the compressor on and off to save voltage
    if(on){
      compressor.enableDigital();
    } else{
      compressor.disable();
    }
  }
}
