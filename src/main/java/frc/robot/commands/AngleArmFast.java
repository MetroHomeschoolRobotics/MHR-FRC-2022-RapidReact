// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
//import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AngleArmFast extends CommandBase {
  /** Creates a new AngleArm. */
  private Arm arm;
  private double setpoint;
  private PIDController armPID = new PIDController(20, 0, 3);
  private boolean end;
  private Debouncer endDeb = new Debouncer(.0625);
  public AngleArmFast(double _setpoint, Arm _arm
  ) {
    armPID.setTolerance(.006);
    arm=_arm;
    setpoint = _setpoint;
    addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPID.calculate(arm.getArmPot(), setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armPID.calculate(arm.getArmPot());
    if(!armPID.atSetpoint()) {
     arm.setArmMotor(armPID.calculate(arm.getArmPot(), setpoint));
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
    end = armPID.atSetpoint();
    return endDeb.calculate(end);
  }
}
