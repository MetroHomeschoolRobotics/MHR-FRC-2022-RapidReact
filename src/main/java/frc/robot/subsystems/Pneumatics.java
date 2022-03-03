// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,RobotMap.intakeSolenoid1, RobotMap.intakeSolenoid2);
  private DoubleSolenoid hookSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.hookSolenoid1, RobotMap.hookSolenoid2);
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      hookSolenoid.set(Value.kForward);
    } else {
      hookSolenoid.set(Value.kReverse);
    }
  }

  public boolean getHook() {
    if(hookSolenoid.get()==Value.kForward) {
      return true;
    } else {
      return false;
    }
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
