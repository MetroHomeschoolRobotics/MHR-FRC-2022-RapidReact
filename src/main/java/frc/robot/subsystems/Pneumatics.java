// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private Compressor compressor = new Compressor( 0, PneumaticsModuleType.CTREPCM);
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
