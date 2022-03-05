// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BlueBallsLHS;
import frc.RedBalls;
import frc.RedBallsLHS;
import frc.robot.BlueBalls;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  public Vision() {
    setUpIntakeVision();
  }

  private UsbCamera intakeCam;
  public static final int IMG_WIDTH = 320;
  public static final int IMG_HEIGHT = 240;
  private VisionThread visionThread;
  private double centerX = 0.0;
  private double centerY = 0.0;
  private final Object imgLock = new Object();

  private double intakeTX = 0;
  private double intakeTY = 0;
  private boolean intakeTV = false;

  /**Intake Vision Pipeline Code */
  private void setUpIntakeVision() {
    intakeCam = CameraServer.startAutomaticCapture(); //Starts USB camera on RIO. 
      intakeCam.setResolution(IMG_WIDTH, IMG_HEIGHT);
        visionThread = new VisionThread(intakeCam, 
        //THIS PIECE MUST CHANGE TO CHANGE PIPELINES!!!!
        //USE blueBallPipeline or redBallPipeline
        new RedBalls(),
        //MAKE SURE TO UPDATE THIS BEFORE EACH MATCH
        pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            centerY = r.y + (r.height / 2);
            intakeTX = r.x+(r.width/2);
            intakeTY=r.y+(r.height/2);
            intakeTV = true;
        }
        } else {
          centerX = IMG_WIDTH/2;
          centerY = IMG_HEIGHT/2;
          intakeTV = false;
        }
        SmartDashboard.putBoolean("TV", intakeTV);
        SmartDashboard.putNumber("centerX", centerX);
        SmartDashboard.putNumber("centerY", centerY);
    });
    visionThread.start();
  }

  public boolean getIntakeHasTarget() {
    return intakeTV;
  }

  public double getIntakeTX() {
    return intakeTX;
  }

  public double getIntakeTY() {
    return intakeTY;
  }

  public boolean getLimelightHasTarget() {
    if(limelight.getEntry("tv").getDouble(0)==1) {
      return true;
    } else {
      return false;
    }
  }
  public double getLimelightTY() {
    return limelight.getEntry("ty").getDouble(0);
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
    return 0;
  };
  public double get_shooter_rps (double target_angle){
    return 0;
  };
  
  @Override public void periodic() {
    
  }
}
