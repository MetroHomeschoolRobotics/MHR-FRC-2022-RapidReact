// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.BlueBalls;
import frc.RedBalls;
import frc.TrajectoryHelper;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private UsbCamera intakeCam;
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;
  private VisionThread visionThread;
  private double centerX = 0.0;
  private double centerY = 0.0;
  private final Object imgLock = new Object();
  private RedBalls redBallPipeline = new RedBalls();
  private BlueBalls blueBallPipeline = new BlueBalls();
  
  /**
   * Constructor
   */
  public RobotContainer() {
    setDefaultCommands();  //Sets the default commands for each subsystem
    setAutoChooserOptions();  //Sets up autonomous routines
    configureButtonBindings();  //Configures button bindings for joysticks
    setUpIntakeVision();
  }

  /**Intake Vision Pipeline Code */
  private void setUpIntakeVision() {
    intakeCam = CameraServer.startAutomaticCapture(); //Starts USB camera on RIO. 
      intakeCam.setResolution(IMG_WIDTH, IMG_HEIGHT);
        visionThread = new VisionThread(intakeCam, blueBallPipeline, pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            centerY = r.y + (r.height / 2);
          }
          SmartDashboard.putBoolean("TV", !pipeline.filterContoursOutput().isEmpty());
          SmartDashboard.putNumber("centerX", centerX);
          SmartDashboard.putNumber("centerY", centerY);
        }
    });
    visionThread.start();
  }
  /**
   * Create joysticks objects. We use xbox controllers. 
   These will be used for button bindings in the configureButtonBindings() method is called. 
   */
  private XboxController _driverController = new XboxController(0);
  //private XboxController _manipulatorController = new XboxController(1);

  /**
   * Define instances of the commands. These are static, so only one instance ever exists. 
   * They are final, so they cannot be overwritten, and they are public,
   *  so we can access them from helper classes. 
   */
  public static final Drivetrain s_drivetrain = new Drivetrain();
  public static final Intake s_intake = new Intake();
  public static final Arm s_arm = new Arm();
  //TODO: comment these back in
  //public static final  Shooter s_shooter = new Shooter();
  //public static final Vision s_vision = new Vision();
  //public static final Pneumatics s_pneumatics = new Pneumatics();
  
  /**
   * These are the commands. Instances of these commands are bound to buttons. 
   * Auto commands should not be stated here because
   * each instance can only be used once in command groups. 
   */
  private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain,_driverController);
  private final RunIntake c_runIntake = new RunIntake(s_intake);
  private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
  //private final SpinShooter c_spinShooter = new SpinShooter(s_shooter, _driverController);
  //private final AimDrivetrain c_aimDrivetrain = new AimDrivetrain(s_vision, s_drivetrain);
  //private final TargetBall c_targetBall = new TargetBall(s_vision, s_drivetrain);
  //private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics); 
  
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
  }

  /**
   * Set autonomous routine options
   * Current options: 
   * 1. Don't move. 
   * 2. Test trajectory code. 
   */
  private void setAutoChooserOptions() {
    _autoChooser.setDefaultOption("No autonomous", new WaitCommand(15));
    _autoChooser.addOption("Test trajectory", TrajectoryHelper.createTrajectoryCommand(
      TrajectoryHelper.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
      List.of(new Translation2d(1,0), new Translation2d(2,1)),
      new Pose2d(3,0, new Rotation2d(0)),
      false, 2, 2, 0, 0, 7
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
    /*
    final JoystickButton bButton = new JoystickButton(_driverController, 2 );
    bButton.whileHeld(c_spinShooter);
    final JoystickButton aButton = new JoystickButton(_driverController, 1);
    aButton.whileHeld(c_aimDrivetrain);
    final JoystickButton yButton = new JoystickButton(_driverController, 4);
    yButton.whileHeld(c_targetBall);
    final JoystickButton startButton = new JoystickButton(_driverController, 8);
    startButton.whenPressed(c_toggleCompressor);
    */
  }
    
    /**
     * This is a method run by {@link Robot} that schedules the autonomous command selected in the dropdown. 
     */
    public Command getAutonomousCommand() {
      return _autoChooser.getSelected();
    }
    
  }
