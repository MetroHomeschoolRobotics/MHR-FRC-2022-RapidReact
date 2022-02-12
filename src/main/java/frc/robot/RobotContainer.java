// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

//import javax.xml.catalog.GroupEntry.PreferType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
 // private final Shooter s_shooter = new Shooter();
  private final Intake s_intake = new Intake();
  //private final Vision s_vision = new Vision();
  private final Arm s_arm = new Arm();
 // private final Pneumatics s_pneumatics = new Pneumatics();
  //Define instances of the commands
  private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain,_driverController);
 // private final SpinShooter c_spinShooter = new SpinShooter(s_shooter, _driverController);
  private final RunIntake c_runIntake = new RunIntake(s_intake);
  private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
  private final TurnToAngle c_turntoangle = new TurnToAngle(0, s_drivetrain);
  private final DriveDistance c_driveDistance = new DriveDistance(s_drivetrain, 60);
  //private final AimDrivetrain c_aimDrivetrain = new AimDrivetrain(s_vision, s_drivetrain);
 // private final TargetBall c_targetBall = new TargetBall(s_vision, s_drivetrain);
  //private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics); 

  
  //Create the autonomous command chooser.
  SendableChooser<Command> _autoChooser = new SendableChooser<>();//creates a menu of commands that we will put on the dashboard. This will enable us to choose our auto routine before matches.  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    init();  
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return _autoChooser.getSelected();
    var leftController = new PIDController(Constants.kP, 0, 0);
    var rightController = new PIDController(Constants.kP, 0, 0);

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ks,
            Constants.kv,
            Constants.ka),
        Constants.kDriveKinematics,
        10);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
            new Pose2d(3, 1, new Rotation2d(Units.degreesToRadians(0))),
        // Pass config
        config);

        s_drivetrain.getField2d().getObject("traj").setTrajectory(exampleTrajectory);

        RamseteController ramseteThing = new RamseteController(Constants.kRamseteB, Constants.kRamseteZ);
RamseteCommand ramseteCommand =
    new RamseteCommand(exampleTrajectory, s_drivetrain::getPose, ramseteThing, new SimpleMotorFeedforward(Constants.ks, Constants.kv,Constants.ka),
        Constants.kDriveKinematics,
        s_drivetrain::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) -> {
          s_drivetrain.tankDriveVolts(leftVolts, rightVolts);
      },
        s_drivetrain);


// Reset odometry to the starting pose of the trajectory.
s_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
// Run path following command, then stop at the end.
return ramseteCommand;//.andThen(() -> s_drivetrain.tankDriveVolts(0, 0));
  }
}
