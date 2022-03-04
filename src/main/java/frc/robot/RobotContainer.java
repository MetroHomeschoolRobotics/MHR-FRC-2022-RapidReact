// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.TrajectoryHelper;
import frc.robot.commands.AngleArm;
import frc.robot.commands.ArmManual;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseMagazine;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunMagazine;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleHook;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.WinchClimber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  /**
   * Constructor
   */
  public RobotContainer() {
    setDefaultCommands();  //Sets the default commands for each subsystem
    setAutoChooserOptions();  //Sets up autonomous routines
    configureButtonBindings();  //Configures button bindings for joystick
  }

  
  /**
   * Create joysticks objects. We use xbox controllers. 
   These will be used for button bindings in the configureButtonBindings() method is called. 
   */
  private XboxController _driverController = new XboxController(0);
  private XboxController _manipulatorController = new XboxController(1);

  /**
   * Define instances of the commands. These are static, so only one instance ever exists. 
   * They are final, so they cannot be overwritten, and they are public,
   *  so we can access them from helper classes. 
   */
  public static final Drivetrain s_drivetrain = new Drivetrain();
  public static final Intake s_intake = new Intake();
  public static final Arm s_arm = new Arm();
  public static final  Shooter s_shooter = new Shooter();
  public static final Vision s_vision = new Vision();
  public static final Pneumatics s_pneumatics = new Pneumatics();
  public static final Magazine s_magazine = new Magazine();
  public static final ClimberWinch s_climber = new ClimberWinch();
  
  /**
   * These are the commands. Instances of these commands are bound to buttons. 
   * Auto commands should not be stated here because
   * each instance can only be used once in command groups. 
   */
  private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain,_driverController);
  private final RunIntake c_runIntake = new RunIntake(s_intake);
  private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
  private final SpinShooter c_spinShooter = new SpinShooter(s_shooter, _driverController);
  private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics); 
  private final ToggleIntake c_toggleIntake = new ToggleIntake(s_pneumatics);
  private final WinchClimber c_winchClimber = new WinchClimber(s_climber, _manipulatorController);
  /**
   * This is a menu displayed on the dashboard that we use to select autonomous routines
   */
  SendableChooser<Command> _autoChooser = new SendableChooser<>();

  /**
   * Set default commands for the subsystems. 
   * Default commands run when a subsystem has no other commands running. 
   * Example: drivetrain should default to joysticks when not being used by automatic driving classes. 
   */
  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(s_drivetrain, c_driveTeleop);
    CommandScheduler.getInstance().setDefaultCommand(s_arm, new ArmManual(s_arm, _manipulatorController));
    CommandScheduler.getInstance().setDefaultCommand(s_climber, c_winchClimber);
  }

  /**
   * Set autonomous routine options
   * Current options: 
   * 1. Don't move. 
   * 2. Test trajectory code. 
   */
  private void setAutoChooserOptions() {
    _autoChooser.setDefaultOption("No autonomous", new WaitCommand(15));
    _autoChooser.addOption("Test trajectory", new ResetOdometry(new Pose2d(0,0,new Rotation2d(0)), s_drivetrain).andThen(TrajectoryHelper.createTrajectoryCommand(
      TrajectoryHelper.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
      List.of(new Translation2d(1,0.1), new Translation2d(2,1)),
      new Pose2d(3,0, new Rotation2d(0)),
      false, 2, 1, 0, 0, 5
      ))
      .andThen(
        TrajectoryHelper.createTrajectoryCommand(TrajectoryHelper.generateTrajectory(new Pose2d(3,0, new Rotation2d(0)), 
        List.of(new Translation2d(2,1), new Translation2d(1,1)),
        new Pose2d(0,1, new Rotation2d(0)),
         true, 2, 1, 0, 0, 5))
      ))
    );
    SmartDashboard.putData("Auto Mode", _autoChooser);
  }

  /**
   * This is where we set button bindings. 
   * See button numbers at http://www.team358.org/files/programming/ControlSystem2015-2019/images/XBoxControlMapping.jpg
   */
  private void configureButtonBindings() {      
    final JoystickButton rightBumper = new JoystickButton(_driverController, 5 );
    rightBumper.whileHeld(c_reverseIntake);
    final JoystickButton leftBumper = new JoystickButton(_driverController, 6 );
    leftBumper.whileHeld(c_runIntake);
    final JoystickButton bButton = new JoystickButton(_driverController, 2 );
    bButton.whileHeld(c_spinShooter);
    
    final JoystickButton rightBumperM = new JoystickButton(_manipulatorController, 5 );
    rightBumperM.whileHeld(new ReverseMagazine(s_magazine));
    final JoystickButton leftBumperM = new JoystickButton(_manipulatorController, 6 );
    leftBumperM.whileHeld(new RunMagazine(s_magazine));
    
    // final JoystickButton aButton = new JoystickButton(_driverController, 1);
    // aButton.whileHeld(c_aimDrivetrain);
    // final JoystickButton yButton = new JoystickButton(_driverController, 4);
    // yButton.whileHeld(c_targetBall);
    final JoystickButton startButton = new JoystickButton(_driverController, 8);
    startButton.whenPressed(c_toggleCompressor);
    final JoystickButton yButton = new JoystickButton(_driverController, 4);
    yButton.whenPressed(c_toggleIntake);
    SmartDashboard.putNumber("Arm Value", .1);
    final POVButton fenderButton = new POVButton(_manipulatorController, 0);
    fenderButton.toggleWhenPressed(new AngleArm(.1, s_arm));
    final POVButton tarmacButton = new POVButton(_manipulatorController, 90);
    tarmacButton.toggleWhenPressed(new AngleArm(.3, s_arm));
    final POVButton stowButton = new POVButton(_manipulatorController, 180);
    stowButton.toggleWhenPressed(new AngleArm(.5, s_arm));

    final JoystickButton hooksButton = new JoystickButton(_manipulatorController, 1);
    hooksButton.whenPressed(new ToggleHook(s_pneumatics));
  }
    
    /**
     * This is a method run by {@link Robot} that schedules the autonomous command selected in the dropdown. 
     */
    public Command getAutonomousCommand() {
      return _autoChooser.getSelected();
    }
    
  }
