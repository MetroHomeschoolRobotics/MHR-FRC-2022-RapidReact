// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AngleArm extends CommandBase {
  /** Creates a new AngleArm. */
  private Arm arm;
  private double setpoint = 0;
  private Vision s_vision;
  private PIDController armPID = new PIDController(15, 0, 1);
  public AngleArm(double _setpoint, Arm _arm, Vision _vision
  ) {
    armPID.setTolerance(.004);
    arm=_arm;
    s_vision = _vision;
    setpoint = 0;
    setpoint = _setpoint;
    addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(setpoint!=0) {
      armPID.calculate(arm.getArmPot(), setpoint);
    } else {
      armPID.calculate(arm.getArmPot(), s_vision.get_arm_angle(s_vision.getLimelightTY()));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!armPID.atSetpoint()) {
      if(setpoint!=0) {
        arm.setArmMotor(armPID.calculate(arm.getArmPot(), setpoint));
      } else {
        
        arm.setArmMotor(armPID.calculate(arm.getArmPot(), s_vision.get_arm_angle(s_vision.getLimelightTY())));
      }
    } else {
      arm.setArmMotor(0);
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armPID.atSetpoint();
  }
}
