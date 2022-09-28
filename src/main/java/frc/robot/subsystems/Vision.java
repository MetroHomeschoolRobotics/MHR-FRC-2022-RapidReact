// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
    SmartDashboard.putNumber("c", .820984);
    //SmartDashboard.putNumber("c", 0.2);
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
    //return -6.9277e-5*Math.pow(target_angle, 2)-.0076533432*target_angle+.3606892849;
    //Target angle is from Limelight TY value - return value is setting the poteniomiter value go to
    //TODO Set potentiomter values and angles
    if(target_angle>=16) { //at limelight camera value ty(16) arm potentiometer value (.43)
      return .43;
    }else if(target_angle>=12) {
      return .44;
    } else if(target_angle>=8) {
      return .45;
    } else if(target_angle>=3) {
      return .47;
    } else if(target_angle>=0) {
      return .48; // was 0.47
    } else if(target_angle>=-5) {
      return .51;
    } else if(target_angle>=-9) {
      return .525;
    } else if(target_angle>=-13) {
      return .53;
    } else if(target_angle>=-18) {
      return .54;
    } else if(target_angle>=-19) {
      return .545;
    } else {
      return .59;
    }
        //return 0;
  };
  public double get_shooter_rps (double target_angle){
    //return SmartDashboard.getNumber("a", 0)*Math.pow(target_angle+30, SmartDashboard.getNumber("b", 0));
    //return (SmartDashboard.getNumber("a", 0)*Math.pow((target_angle+30)+SmartDashboard.getNumber("c", 0), SmartDashboard.getNumber("b", 0)))+SmartDashboard.getNumber("d", 0);
    if(target_angle < -20) {
      return MathUtil.interpolate(Constants.shooterAtm20, Constants.shooterAtm23, -(target_angle+20)/3);
    } else if(target_angle < -17) {
      return MathUtil.interpolate(Constants.shooterAtm17, Constants.shooterAtm20, -(target_angle+17)/3);
    } else if(target_angle < -14) {
      return MathUtil.interpolate(Constants.shooterAtm14, Constants.shooterAtm17, -(target_angle+14)/3);
    } else if(target_angle < -11) {
      return MathUtil.interpolate(Constants.shooterAtm11, Constants.shooterAtm14, -(target_angle+11)/3);
    } else if(target_angle < -8) {
      return MathUtil.interpolate(Constants.shooterAtm8, Constants.shooterAtm11, -(target_angle+8)/3);
    } else if(target_angle < -5) {
      return MathUtil.interpolate(Constants.shooterAtm5, Constants.shooterAtm8, -(target_angle+5)/3);
    } else if(target_angle < -2) {
      return MathUtil.interpolate(Constants.shooterAtm2, Constants.shooterAtm5, -(target_angle+2)/3);
    } else if(target_angle < 0) {
      return MathUtil.interpolate(Constants.shooterAtm2, Constants.shooterAt0, (target_angle+2)/2);
    } else if(target_angle < 2) {
      return MathUtil.interpolate(Constants.shooterAt0, Constants.shooterAt2, (target_angle-0)/2);
    } else if(target_angle < 5) {
      return MathUtil.interpolate(Constants.shooterAt2, Constants.shooterAt5, (target_angle-2)/3);
    } else if(target_angle < 8) {
      return MathUtil.interpolate(Constants.shooterAt5, Constants.shooterAt8, (target_angle-5)/3);
    } else if(target_angle < 11) {
      return MathUtil.interpolate(Constants.shooterAt8, Constants.shooterAt11, (target_angle-8)/3);
    } else if(target_angle < 14) {
      return MathUtil.interpolate(Constants.shooterAt11, Constants.shooterAt14, (target_angle-11)/3);
    } else if(target_angle < 17) {
      return MathUtil.interpolate(Constants.shooterAt14, Constants.shooterAt17, (target_angle-14)/3);
    } else if(target_angle < 20) {
      return MathUtil.interpolate(Constants.shooterAt17, Constants.shooterAt20, (target_angle-17)/3);
    } else {
      return 0;
    }
  };
  
  @Override public void periodic() {
    
  }
}
