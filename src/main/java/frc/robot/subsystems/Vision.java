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
  private PhotonCamera intakeCamera = new PhotonCamera("IntakeCam");
  private PhotonCamera limelight = new PhotonCamera("limelight");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("target visible", limelight.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("IntakeHasTarget", intakeHasTarget());
  }

  public void setLimelightLEDS(int state) { //0 = off; 1 = on; 2 = blink
    if(state == 0) {
      limelight.setLED(VisionLEDMode.kOff);
    } else if(state ==1) {
      limelight.setLED(VisionLEDMode.kOn);
    } else if(state == 2) {
      limelight.setLED(VisionLEDMode.kBlink);
    }
  }

  public PhotonPipelineResult getIntakeCameraResult() {
    return intakeCamera.getLatestResult();
  }

  public void setLimelightDriverMode(boolean state) {
    limelight.setDriverMode(state);
  }

  public void setIntakeDriverMode(boolean state) {
    intakeCamera.setDriverMode(state);
  }

  public PhotonPipelineResult getLimelightResult() {
    return limelight.getLatestResult();
  }

  public PhotonTrackedTarget getLimelightTarget() {
    return limelight.getLatestResult().getBestTarget();
  }

  public PhotonTrackedTarget getIntakeTarget() {
    return intakeCamera.getLatestResult().getBestTarget();
  }

  public boolean intakeHasTarget() {
    return intakeCamera.getLatestResult().hasTargets();
  }
  
  public boolean limelightHasTarget() {
    return limelight.getLatestResult().hasTargets();
  }
}
