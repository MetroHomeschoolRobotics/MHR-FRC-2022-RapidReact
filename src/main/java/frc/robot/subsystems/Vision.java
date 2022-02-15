// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.*;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}
  //Create photonvision camera objects
  private PhotonCamera intakeCamera = new PhotonCamera("IntakeCam");
  private PhotonCamera limelight = new PhotonCamera("limelight");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Tell the dashboard if each of the cameras has a target
    SmartDashboard.putBoolean("target visible", limelight.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("IntakeHasTarget", intakeHasTarget());
  }

  //toggle lights on the limelight to not blind people
  public void setLimelightLEDS(int state) { //0 = off; 1 = on; 2 = blink
    if(state == 0) {
      limelight.setLED(VisionLEDMode.kOff);
    } else if(state ==1) {
      limelight.setLED(VisionLEDMode.kOn);
    } else if(state == 2) {
      limelight.setLED(VisionLEDMode.kBlink);
    }
  }

  //gets image processing results from each camera
  public PhotonPipelineResult getIntakeCameraResult() {
    return intakeCamera.getLatestResult();
  }
  public PhotonPipelineResult getLimelightResult() {
    return limelight.getLatestResult();
  }

  //Set cameras to driver mode (switches them to color and disables processing)
  public void setLimelightDriverMode(boolean state) {
    limelight.setDriverMode(state);
  }
  public void setIntakeDriverMode(boolean state) {
    intakeCamera.setDriverMode(state);
  }

  //Get target object from each camera
  public PhotonTrackedTarget getLimelightTarget() {
    return limelight.getLatestResult().getBestTarget();
  }
  public PhotonTrackedTarget getIntakeTarget() {
    return intakeCamera.getLatestResult().getBestTarget();
  }

  //get if each camera has a target in sight
  public boolean intakeHasTarget() {
    return intakeCamera.getLatestResult().hasTargets();
  }
  public boolean limelightHasTarget() {
    return limelight.getLatestResult().hasTargets();
  }
  
}
