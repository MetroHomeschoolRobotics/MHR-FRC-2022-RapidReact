// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;

public class AngleArmLL extends CommandBase {
  /** Creates a new AngleArm. */
  private Arm arm;
  private Vision s_vision;
  private static PIDController armPID = new PIDController(15, 0, 1);
  public AngleArmLL(Arm _arm, Vision _vision
  ) {
    armPID.setTolerance(.004);
    arm=_arm;
    s_vision = _vision;
    addRequirements(_arm);
    armPID.calculate(arm.getArmPot(), s_vision.get_arm_angle(s_vision.getLimelightTY()));
    //SmartDashboard.putData(armPID);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      armPID.calculate(arm.getArmPot(), s_vision.get_arm_angle(s_vision.getLimelightTY()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        arm.setArmMotor(armPID.calculate(arm.getArmPot(), s_vision.get_arm_angle(s_vision.getLimelightTY())));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getArmPot()-s_vision.get_arm_angle(s_vision.getLimelightTY()))<.03;
  }
}
