// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Create joystick objects. Our team uses two Xbox controllers. Xbox controllers are given port numbers by the driver station software
  //Our convention is that port 0 is used for the driver controller and port 1 is used for the manipulator driver controller. (sortof like a weapons specialist)
  private XboxController _driverController = new XboxController(0);
  //private XboxController _manipulatorController = new XboxController(1);

  //Define instances of the subsystem classes (final means that the object the variable refers to is unchangeable, but the data in the object is.)
  //we will use s_ as a prefix to designate subsystems and c_ as a prefix to designate commands. 
  private final Drivetrain s_drivetrain = new Drivetrain();
  private final Shooter s_shooter = new Shooter();
  private final Intake s_intake = new Intake();
  //Define instances of the commands
  private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain,_driverController);
  private final SpinShooter c_spinShooter = new SpinShooter(s_shooter);
  private final RunIntake c_runIntake = new RunIntake(s_intake);
  private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
  private final TurnToAngle c_turntoangle = new TurnToAngle(0, s_drivetrain);
  
  //Create the autonomous command chooser.
  SendableChooser<Command> _autoChooser = new SendableChooser<>();//creates a menu of commands that we will put on the dashboard. This will enable us to choose our auto routine before matches.  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    init();  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(s_drivetrain, c_driveTeleop);
  }
  private void setAutoChooserOptions() {
    _autoChooser.setDefaultOption("No autonomous", new WaitCommand(15));
  }

//view joystick button numbers at http://www.team358.org/files/programming/ControlSystem2015-2019/images/XBoxControlMapping.jpg
  private void configureButtonBindings() {   
    SmartDashboard.putNumber("desired angle", 0);   
    final JoystickButton rightBumper = new JoystickButton(_driverController, 5 );
    rightBumper.whileHeld(c_runIntake);
    final JoystickButton leftBumper = new JoystickButton(_driverController, 6 );
    leftBumper.whileHeld(c_reverseIntake);
    final JoystickButton aButton = new JoystickButton(_driverController, 1 );
    aButton.whileHeld(c_spinShooter);
    SmartDashboard.putData("turn to 0",c_turntoangle);
  }

  private void init() {
    setDefaultCommands();
    setAutoChooserOptions();
    configureButtonBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
  }
}
