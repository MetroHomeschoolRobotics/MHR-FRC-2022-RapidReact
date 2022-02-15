// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

//import javax.xml.catalog.GroupEntry.PreferType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Intake s_intake = new Intake();
  private final Arm s_arm = new Arm();
  //TODO: comment these back in
  //private final Shooter s_shooter = new Shooter();
  //private final Vision s_vision = new Vision();
  //private final Pneumatics s_pneumatics = new Pneumatics();
  
  //Define instances of the commands
  private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain,_driverController);
  private final RunIntake c_runIntake = new RunIntake(s_intake);
  private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
  private final TurnToAngle c_turntoangle = new TurnToAngle(0, s_drivetrain);
  private final DriveDistance c_driveDistance = new DriveDistance(s_drivetrain, 60);
  // private final SpinShooter c_spinShooter = new SpinShooter(s_shooter, _driverController);
  //private final AimDrivetrain c_aimDrivetrain = new AimDrivetrain(s_vision, s_drivetrain);
 // private final TargetBall c_targetBall = new TargetBall(s_vision, s_drivetrain);
  //private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics); 

  DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka), Constants.kDriveKinematics, 10);
  TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);
  
  //Create the autonomous command chooser.
  SendableChooser<Command> _autoChooser = new SendableChooser<>();//creates a menu of commands that we will put on the dashboard. This will enable us to choose our auto routine before matches.  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    init();  
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //s_vision.setLimelightLEDS(0);
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
    _autoChooser.addOption("Test traj", createTrajectoryCommand(
      TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config)
    ));
    _autoChooser.addOption("2 Ball", new SequentialCommandGroup(
      new ParallelRaceGroup(new RunIntake(s_intake), new DriveDistance(s_drivetrain, 120)),
      new DriveDistance(s_drivetrain, -120)//insert limelight command
     // new ParallelRaceGroup(new WaitCommand(5), new SpinShooter(s_shooter, _driverController), new RunIntake(s_intake))
      ));

      SmartDashboard.putData("Auto Mode", _autoChooser);
  }

//view joystick button numbers at http://www.team358.org/files/programming/ControlSystem2015-2019/images/XBoxControlMapping.jpg
  private void configureButtonBindings() {   
    SmartDashboard.putNumber("desired angle", 0);   
    final JoystickButton rightBumper = new JoystickButton(_driverController, 5 );
    rightBumper.whileHeld(c_reverseIntake);
    final JoystickButton leftBumper = new JoystickButton(_driverController, 6 );
    leftBumper.whileHeld(c_runIntake);
    final JoystickButton bButton = new JoystickButton(_driverController, 2 );
   // bButton.whileHeld(c_spinShooter);
    final JoystickButton aButton = new JoystickButton(_driverController, 1);
   // aButton.whileHeld(c_aimDrivetrain);
    final JoystickButton yButton = new JoystickButton(_driverController, 4);
   // yButton.whileHeld(c_targetBall);
    final JoystickButton startButton = new JoystickButton(_driverController, 8);
    //startButton.whenPressed(c_toggleCompressor);
    SmartDashboard.putData("turn to 0",c_turntoangle);
    SmartDashboard.putData("drive one foot",c_driveDistance);
  }

  private void init() {
    setDefaultCommands();
    setAutoChooserOptions();
    configureButtonBindings();
  }


  public Command createTrajectoryCommand(Trajectory _trajectoryToFollow) {
    //Trajectory _trajectoryToFollow = PathPlanner.loadPath(trajectoryName, maxVel, maxAccel);
    var leftController = new PIDController(Constants.kP, 0, 0);
    var rightController = new PIDController(Constants.kP, 0, 0);
    s_drivetrain.getField2d().getObject("traj").setTrajectory(_trajectoryToFollow);
    RamseteController ramseteThing = new RamseteController(Constants.kRamseteB, Constants.kRamseteZ);
    RamseteCommand ramseteCommand = new RamseteCommand(
      _trajectoryToFollow,
      s_drivetrain::getPose, 
      ramseteThing, 
      new SimpleMotorFeedforward(Constants.ks, Constants.kv,Constants.ka),
      Constants.kDriveKinematics,
      s_drivetrain::getWheelSpeeds,
      leftController,
      rightController,
      (leftVolts, rightVolts) -> {
        s_drivetrain.tankDriveVolts(leftVolts, rightVolts);
      },
      s_drivetrain);
      s_drivetrain.resetOdometry(_trajectoryToFollow.getInitialPose());
      return new SequentialCommandGroup(new ResetOdometry(_trajectoryToFollow.getInitialPose(), s_drivetrain), ramseteCommand);
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
