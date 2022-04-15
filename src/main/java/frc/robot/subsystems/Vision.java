// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private UsbCamera intakeCamera = new UsbCamera("USB Camera 0", 0);
  private MjpegServer mjpegServer1 = new MjpegServer("Intake Camera", 1181);

  private double tY = 0;

  public Vision() {
    //setUpIntakeVision();
    mjpegServer1.setSource(intakeCamera);
    mjpegServer1.setResolution(160,120);
    mjpegServer1.setFPS(10);
    CameraServer.putVideo("Intake Camera", 160, 120);
    // SmartDashboard.putNumber("a", 7750);
    // SmartDashboard.putNumber("b", -.237);
    
    SmartDashboard.putNumber("a", 8072.3);
    SmartDashboard.putNumber("b", -.258532);
    //SmartDashboard.putNumber("c", .820984);
    SmartDashboard.putNumber("c", 0.2);
    SmartDashboard.putNumber("d", 133.902);




    setPIP(2);
    
  }


  public void setPIP(int number) {
    //limelight.getEntry("stream").setNumber(number);
  }


  public boolean getLimelightHasTarget() {
    return limelight.getEntry("tv").getDouble(0)==1;
  }
  public double getLimelightTY() {
      tY = limelight.getEntry("ty").getDouble(0)/Math.cos(-Units.degreesToRadians(limelight.getEntry("tx").getDouble(0)));
    return tY;
  }
  public double getLimelightTX() {
    return limelight.getEntry("tx").getDouble(0);
  }
  public void setLights(boolean on) {
    if(on) {
      limelight.getEntry("ledMode").setNumber(3);
    } else {
      limelight.getEntry("ledMode").setNumber(1);
    }
  }

  public boolean getLights() {
    if(limelight.getEntry("ledMode").getDouble(0)==3) {
      return true;
    } else {
      return false;
    }
  }

  public void setMode(boolean processing) {
    if(processing) {
      limelight.getEntry("camMode").setNumber(0);
    } else {
      limelight.getEntry("camMode").setNumber(1);
    }
  } 
  public void setPipeline(int number) {
    limelight.getEntry("pipeline").setNumber(number);
  }

  public double get_arm_angle (double target_angle){ 
    //return (1.7376954e-6)*(Math.pow(target_angle, 3))+1.545765e-4*(Math.pow(target_angle,2))+-.0030598857*(target_angle)+.2092726891;
    //return 1.2831802e-6*Math.pow(target_angle, 4)+2.2548459e-5*Math.pow(target_angle, 3)-2.559467e-4*Math.pow(target_angle, 2)-.0102652388*target_angle+.3654017083;
    return -6.9277e-5*Math.pow(target_angle, 2)-.0076533432*target_angle+.3606892849;
    /*if(target_angle>=12) {
      return .25;
    }else if(target_angle>=8) {
      return .28;
    } else if(target_angle>=3) {
      return .33;
    } else if(target_angle>=0) {
      return .38;
    } else if(target_angle>=-13) {
      return .44;
    } else if(target_angle>=-18) {
      return .45;
    } else if(target_angle>=-19) {
      return .5;
    } else {
      return .5;
    }*/
    //return 0;
  };
  public double get_shooter_rps (double target_angle){
    //return SmartDashboard.getNumber("a", 0)*Math.pow(target_angle+30, SmartDashboard.getNumber("b", 0));
    return (SmartDashboard.getNumber("a", 0)*Math.pow((target_angle+30)+SmartDashboard.getNumber("c", 0), SmartDashboard.getNumber("b", 0)))+SmartDashboard.getNumber("d", 0);
  };
  
  @Override public void periodic() {
    
  }
}
