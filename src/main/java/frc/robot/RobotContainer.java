// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.TrajectoryHelper;
import frc.robot.commands.AngleArm;
import frc.robot.commands.AngleArmFast;
import frc.robot.commands.AngleArmLL;
import frc.robot.commands.AngleArmSlow;
import frc.robot.commands.ApplyClimberPower;
import frc.robot.commands.ArmManual;
import frc.robot.commands.AutoMagazine;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.PrepareMagazineToShoot;
import frc.robot.commands.ResetClimberEncoder;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseMagazine;
import frc.robot.commands.RunClimberTillLimit;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunMagazine;
import frc.robot.commands.SetClimbPistons;
import frc.robot.commands.SpinArmAtSDRPM;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleHook;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleLights;
import frc.robot.commands.WinchClimber;
import frc.robot.commands.WinchClimberRelative;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        /**
         * Constructor
         */
        public RobotContainer() {
                setDefaultCommands(); // Sets the default commands for each subsystem
                setAutoChooserOptions(); // Sets up autonomous routines
                configureButtonBindings(); // Configures button bindings for joystick
                SmartDashboard.putNumber("shooter RPM given angle", 0);
                LiveWindow.disableAllTelemetry();// Will this fix loop overruns?
        }

        /**
         * Create joysticks objects. We use xbox controllers.
         * These will be used for button bindings in the configureButtonBindings()
         * method is called.
         */
        public XboxController driverController = new XboxController(0);
        public XboxController manipulatorController = new XboxController(1);
        public Joystick tableStick = new Joystick(2);

        /**
         * Define instances of the commands. These are static, so only one instance ever
         * exists.
         * They are final, so they cannot be overwritten, and they are public,
         * so we can access them from helper classes.
         */
        public static double armFromLimelight = 0;
        public static double ArmAngleReturnAfterMacro = 0.5; // added this variable instead of hard coded numbers.
        public static final Drivetrain s_drivetrain = new Drivetrain();
        public static final Arm s_arm = new Arm();
        public static final Shooter s_shooter = new Shooter();
        public static final Vision s_vision = new Vision();
        public static final Pneumatics s_pneumatics = new Pneumatics();
        public static final Intake s_intake = new Intake(s_pneumatics);
        public static final Magazine s_magazine = new Magazine();
        public static final ClimberWinch s_climber = new ClimberWinch();

        /**
         * These are the commands. Instances of these commands are bound to buttons.
         * Auto commands should not be stated here because
         * each instance can only be used once in command groups.
         */
        private final DriveTeleop c_driveTeleop = new DriveTeleop(s_drivetrain, driverController);
        // private final RunIntake c_runIntake = new RunIntake(s_intake);
        private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics);
        private final ToggleIntake c_toggleIntake = new ToggleIntake(s_pneumatics);
        private final WinchClimber c_winchClimber = new WinchClimber(s_climber, manipulatorController,
                        driverController);

        /**
         * This is a menu displayed on the dashboard that we use to select autonomous
         * routines
         */
        SendableChooser<Command> _autoChooser = new SendableChooser<>();

        /**
         * Set default commands for the subsystems.
         * Default commands run when a subsystem has no other commands running.
         * Example: drivetrain should default to joysticks when not being used by
         * automatic driving classes.
         */
        private void setDefaultCommands() {
                CommandScheduler.getInstance().setDefaultCommand(s_drivetrain, c_driveTeleop);
                CommandScheduler.getInstance().setDefaultCommand(s_arm, new ArmManual(s_arm, manipulatorController));
                CommandScheduler.getInstance().setDefaultCommand(s_climber, c_winchClimber);
                CommandScheduler.getInstance().setDefaultCommand(s_magazine,
                                new AutoMagazine(s_magazine, s_intake, s_shooter, driverController));
        }

        /**
         * Set autonomous routine options
         * Current options:
         * 1. Don't move.
         * 2. Test trajectory code.
         */
        private void setAutoChooserOptions() {
                _autoChooser.setDefaultOption("No autonomous", new WaitCommand(15));

                _autoChooser.addOption("Test trajectory",
                                new ResetOdometry(new Pose2d(0, 0, new Rotation2d(0)), s_drivetrain)
                                                .andThen(TrajectoryHelper.createTrajectoryCommand(
                                                                TrajectoryHelper.generateTrajectory(
                                                                                new Pose2d(0, 0, new Rotation2d(0)),
                                                                                List.of(new Translation2d(1, 0.1),
                                                                                                new Translation2d(2,
                                                                                                                1)),
                                                                                new Pose2d(5, 0, new Rotation2d(0)),
                                                                                false, 3, 2, 1, 0, 0, 7))
                                                                .andThen(new WaitCommand(1)
                                                                                .andThen(
                                                                                                TrajectoryHelper.createTrajectoryCommand(
                                                                                                                TrajectoryHelper.generateTrajectory(
                                                                                                                                new Pose2d(3, 0, new Rotation2d(
                                                                                                                                                0)),
                                                                                                                                List.of(new Translation2d(
                                                                                                                                                2,
                                                                                                                                                1),
                                                                                                                                                new Translation2d(
                                                                                                                                                                1,
                                                                                                                                                                0)),
                                                                                                                                new Pose2d(0, 0, new Rotation2d(
                                                                                                                                                0)),
                                                                                                                                true,
                                                                                                                                3,
                                                                                                                                1,
                                                                                                                                1,
                                                                                                                                0,
                                                                                                                                0,
                                                                                                                                7))))));

                // _autoChooser.addOption("5 ball (right fender)", new
                // ToggleIntake(s_pneumatics).andThen(
                // new ResetOdometry(Constants.threeIntakingR.sample(0).poseMeters,
                // s_drivetrain).andThen(
                // new AngleArm(.27, s_arm, s_vision).alongWith(new
                // PrepareMagazineToShoot(s_magazine, s_intake)).andThen(
                // new ParallelCommandGroup(
                // new SpinShooter(s_shooter, manipulatorController, 3100, s_vision),
                // new RunIntake(s_intake),
                // new RunMagazine(s_magazine, .6)
                // ).withTimeout(.5)).andThen(
                // new RunIntake(s_intake).withTimeout(4.6).andThen(new
                // WaitCommand(0).andThen(new PrepareMagazineToShoot(s_magazine,
                // s_intake)).raceWith(TrajectoryHelper.createTrajectoryCommand(Constants.threeIntakingR)).andThen(
                // new ParallelCommandGroup(
                // new AngleArmLL(s_arm, s_vision),
                // new SpinShooter(s_shooter, manipulatorController, 3400, s_vision),
                // new RunMagazine(s_magazine, .6)//,
                // ).withTimeout(1).andThen(
                // (((new RunIntake(s_intake).alongWith(new RunMagazine(s_magazine,
                // .6))).withTimeout(4)).andThen((new ReverseMagazine(s_magazine,
                // s_shooter).alongWith(new
                // ReverseIntake(s_intake))).withTimeout(.15).andThen(new
                // WaitCommand(10))).raceWith(TrajectoryHelper.createTrajectoryCommand(Constants.fiveballFrom3Ball)).andThen(
                // new ParallelCommandGroup(
                // new AngleArmLL(s_arm, s_vision),
                // new SpinShooter(s_shooter, manipulatorController, 3400, s_vision),
                // new RunIntake(s_intake),
                // new RunMagazine(s_magazine, .4)
                // ).withTimeout(1.5)
                // )
                // )
                // )
                // )
                // )
                // )
                // )
                // ));

                _autoChooser.addOption("5 ball (right fender)", new ToggleIntake(s_pneumatics).andThen(
                                new ResetOdometry(Constants.threeIntakingR.sample(0).poseMeters, s_drivetrain)
                                                /*
                                                 * .andThen(
                                                 * new AngleArm(.27, s_arm, s_vision).alongWith(new
                                                 * PrepareMagazineToShoot(s_magazine)).andThen(
                                                 * new ParallelCommandGroup(
                                                 * new SpinShooter(s_shooter, manipulatorController, 3100, s_vision),
                                                 * new RunIntake(s_intake),
                                                 * new RunMagazine(s_magazine, 1)
                                                 * ).withTimeout(.5)
                                                 */.andThen(
                                                                (new AutoMagazine(s_magazine, s_intake, s_shooter,
                                                                                driverController).alongWith(
                                                                                                new AngleArmSlow(.35,
                                                                                                                s_arm))
                                                                                                .withTimeout(4.25)
                                                                                                .andThen(new PrepareMagazineToShoot(
                                                                                                                s_magazine,
                                                                                                                s_intake).andThen(
                                                                                                                                /*
                                                                                                                                 * new
                                                                                                                                 * SpinShooter
                                                                                                                                 * (
                                                                                                                                 * s_shooter,
                                                                                                                                 * manipulatorController,
                                                                                                                                 * 3800,
                                                                                                                                 * s_vision)
                                                                                                                                 */new WaitCommand(
                                                                                                                                                10)))).raceWith(
                                                                                                                                                                TrajectoryHelper.createTrajectoryCommand(
                                                                                                                                                                                Constants.threeIntakingR))
                                                                                                                                                                .andThen(
                                                                                                                                                                                (new AngleArmSlow(
                                                                                                                                                                                                .4,
                                                                                                                                                                                                s_arm).raceWith(new PrepareMagazineToShoot(s_magazine, s_intake)).andThen(new SpinShooter(s_shooter, manipulatorController, 3500, s_vision).alongWith(
                                                                                                                                                                                                                new ParallelCommandGroup(
                                                                                                                                                                                                                                new AngleArmLL(s_arm,
                                                                                                                                                                                                                                                s_vision),
                                                                                                                                                                                                                                new PrepareMagazineToShoot(
                                                                                                                                                                                                                                                s_magazine,
                                                                                                                                                                                                                                                s_intake).andThen(
                                                                                                                                                                                                                                                                new LimelightAim(
                                                                                                                                                                                                                                                                                s_drivetrain,
                                                                                                                                                                                                                                                                                s_vision,
                                                                                                                                                                                                                                                                                driverController)
                                                                                                                                                                                                                                                                                                .withTimeout(.5)))
                                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                                new RunMagazine(s_magazine,
                                                                                                                                                                                                                                                                                                                                                .45).alongWith(
                                                                                                                                                                                                                                                                                                                                                                new WaitCommand(.2)
                                                                                                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                                                                                                new RunIntake(s_intake))))))
                                                                                                                                                                                                                .withTimeout(2.5)
                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                (new AutoMagazine(
                                                                                                                                                                                                                                                s_magazine,
                                                                                                                                                                                                                                                s_intake,
                                                                                                                                                                                                                                                s_shooter,
                                                                                                                                                                                                                                                driverController)
                                                                                                                                                                                                                                                                .withTimeout(5.5)
                                                                                                                                                                                                                                                                .andThen(new PrepareMagazineToShoot(
                                                                                                                                                                                                                                                                                s_magazine,
                                                                                                                                                                                                                                                                                s_intake).andThen(
                                                                                                                                                                                                                                                                                                new WaitCommand(60))))
                                                                                                                                                                                                                                                                                                                .raceWith(
                                                                                                                                                                                                                                                                                                                                TrajectoryHelper.createTrajectoryCommand(
                                                                                                                                                                                                                                                                                                                                                Constants.fiveballFrom3Ball1)
                                                                                                                                                                                                                                                                                                                                                .andThen(new WaitCommand(
                                                                                                                                                                                                                                                                                                                                                                .5).andThen(TrajectoryHelper.createTrajectoryCommand(Constants.fiveballFrom3Ball2))))
                                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                                new AngleArmSlow(
                                                                                                                                                                                                                                                                                                                                                .4,
                                                                                                                                                                                                                                                                                                                                                s_arm).raceWith(new PrepareMagazineToShoot(s_magazine, s_intake)).andThen(new SpinShooter(s_shooter, manipulatorController, 3600, s_vision).alongWith(
                                                                                                                                                                                                                                                                                                                                                                new ParallelCommandGroup(
                                                                                                                                                                                                                                                                                                                                                                                new AngleArmLL(s_arm,
                                                                                                                                                                                                                                                                                                                                                                                                s_vision),
                                                                                                                                                                                                                                                                                                                                                                                new PrepareMagazineToShoot(
                                                                                                                                                                                                                                                                                                                                                                                                s_magazine,
                                                                                                                                                                                                                                                                                                                                                                                                s_intake).andThen(
                                                                                                                                                                                                                                                                                                                                                                                                                new LimelightAim(
                                                                                                                                                                                                                                                                                                                                                                                                                                s_drivetrain,
                                                                                                                                                                                                                                                                                                                                                                                                                                s_vision,
                                                                                                                                                                                                                                                                                                                                                                                                                                driverController)
                                                                                                                                                                                                                                                                                                                                                                                                                                                .withTimeout(.5)))
                                                                                                                                                                                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                new RunMagazine(s_magazine,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                .45).alongWith(
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                new WaitCommand(.2)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                new RunIntake(s_intake)))))))))))));
                // ));

                _autoChooser.addOption("two ball + steal",
                                new ResetOdometry(Constants.twobS1.sample(0).poseMeters, s_drivetrain)
                                                .andThen(new ToggleIntake(s_pneumatics).andThen(new RunIntake(s_intake)
                                                                .alongWith(new AutoMagazine(s_magazine, s_intake,
                                                                                s_shooter, driverController))
                                                                .raceWith(TrajectoryHelper
                                                                                .createTrajectoryCommand(
                                                                                                Constants.twobS1))
                                                                .andThen(
                                                                                (new AngleArmLL(s_arm, s_vision)
                                                                                                .raceWith(new PrepareMagazineToShoot(
                                                                                                                s_magazine,
                                                                                                                s_intake))
                                                                                                .andThen(new SpinShooter(
                                                                                                                s_shooter,
                                                                                                                manipulatorController,
                                                                                                                3650,
                                                                                                                s_vision).alongWith(
                                                                                                                                new ParallelCommandGroup(
                                                                                                                                                new AngleArmLL(s_arm,
                                                                                                                                                                s_vision),
                                                                                                                                                new PrepareMagazineToShoot(
                                                                                                                                                                s_magazine,
                                                                                                                                                                s_intake).andThen(
                                                                                                                                                                                new LimelightAim(
                                                                                                                                                                                                s_drivetrain,
                                                                                                                                                                                                s_vision,
                                                                                                                                                                                                driverController)
                                                                                                                                                                                                                .withTimeout(.5)))
                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                new RunMagazine(s_magazine,
                                                                                                                                                                                                                                                                .45).alongWith(
                                                                                                                                                                                                                                                                                new WaitCommand(.2)
                                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                                new RunIntake(s_intake))))))
                                                                                                .withTimeout(3).andThen(
                                                                                                                (new RunIntake(s_intake)
                                                                                                                                .alongWith(new RunMagazine(
                                                                                                                                                s_magazine,
                                                                                                                                                0))).raceWith(
                                                                                                                                                                TrajectoryHelper.createTrajectoryCommand(
                                                                                                                                                                                Constants.twobS2))
                                                                                                                                                                .andThen(
                                                                                                                                                                                new ReverseIntake(
                                                                                                                                                                                                s_intake).alongWith(
                                                                                                                                                                                                                new ReverseMagazine(
                                                                                                                                                                                                                                s_magazine,
                                                                                                                                                                                                                                s_shooter))
                                                                                                                                                                                                                .alongWith(new AngleArm(
                                                                                                                                                                                                                                ArmAngleReturnAfterMacro,
                                                                                                                                                                                                                                s_arm,
                                                                                                                                                                                                                                s_vision)))))))));

                _autoChooser.addOption("one ball + taxi", new WaitCommand(1).andThen(
                                (new AngleArmLL(s_arm, s_vision)
                                                .raceWith(new PrepareMagazineToShoot(s_magazine, s_intake))
                                                .andThen(new SpinShooter(s_shooter, manipulatorController, 0, s_vision)
                                                                .alongWith(
                                                                                new ParallelCommandGroup(new AngleArmLL(
                                                                                                s_arm, s_vision),
                                                                                                new PrepareMagazineToShoot(
                                                                                                                s_magazine,
                                                                                                                s_intake).andThen(
                                                                                                                                new LimelightAim(
                                                                                                                                                s_drivetrain,
                                                                                                                                                s_vision,
                                                                                                                                                driverController)
                                                                                                                                                                .withTimeout(.5)))
                                                                                                                                                                                .andThen(
                                                                                                                                                                                                new RunMagazine(s_magazine,
                                                                                                                                                                                                                .45).alongWith(
                                                                                                                                                                                                                                new WaitCommand(.2)
                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                new RunIntake(s_intake)))))))
                                                                                                                                                                                                                                                                                .withTimeout(2)
                                                                                                                                                                                                                                                                                .andThen(
                                                                                                                                                                                                                                                                                                new WaitCommand(10)
                                                                                                                                                                                                                                                                                                                .andThen(new DriveDistance(
                                                                                                                                                                                                                                                                                                                                s_drivetrain,
                                                                                                                                                                                                                                                                                                                                3.5)))));

                _autoChooser.addOption("Spit preload", new ToggleIntake(s_pneumatics)
                                .andThen(new ReverseIntake(s_intake).alongWith(new RunMagazine(s_magazine, -1))));

                _autoChooser.addOption("Two ball straight back",
                                new ToggleIntake(s_pneumatics).andThen(
                                                new RunIntake(s_intake).raceWith(
                                                                new DriveDistance(s_drivetrain, 3.5)).andThen(
                                                                                new DriveDistance(s_drivetrain, -3.5))
                                                                .andThen(new ParallelCommandGroup(
                                                                                new SpinShooter(s_shooter,
                                                                                                manipulatorController,
                                                                                                3500, s_vision),
                                                                                new RunIntake(s_intake),
                                                                                new AngleArm(.3, s_arm, s_vision),
                                                                                new RunMagazine(s_magazine, .6))
                                                                                                .withTimeout(3))));

                SmartDashboard.putData("Auto Mode", _autoChooser);
        }

        /**
         * This is where we set button bindings.
         * See button numbers at
         * http://www.team358.org/files/programming/ControlSystem2015-2019/images/XBoxControlMapping.jpg
         */
        private void configureButtonBindings() {
                // final JoystickButton rightBumper = new JoystickButton(driverController, 5);
                // final JoystickButton aButton = new JoystickButton(driverController, 1);

                // final JoystickButton leftBumper = new JoystickButton(driverController, 6);
                // leftBumper.whileHeld(c_runIntake);

                final JoystickButton rightBumperM = new JoystickButton(manipulatorController, 5);
                rightBumperM.whileHeld(
                                new ReverseMagazine(s_magazine, s_shooter).alongWith(new ReverseIntake(s_intake)));
                final JoystickButton leftBumperM = new JoystickButton(manipulatorController, 6);
                leftBumperM.whileHeld(new RunMagazine(s_magazine, .6).alongWith(new RunIntake(s_intake)));
                // driver shot button
                final JoystickButton bbutton = new JoystickButton(driverController, 2);
                bbutton.whileHeld(new PrepareMagazineToShoot(s_magazine, s_intake).andThen((new SpinShooter(s_shooter,
                                driverController, 0, s_vision)).raceWith(new ParallelCommandGroup(
                                                new LimelightAim(s_drivetrain, s_vision, manipulatorController),
                                                new AngleArmLL(s_arm, s_vision))).andThen(
                                                                new ParallelCommandGroup(
                                                                                new SpinShooter(s_shooter,
                                                                                                driverController, 0,
                                                                                                s_vision),
                                                                                new WaitCommand(.4).andThen(
                                                                                                new ParallelCommandGroup(
                                                                                                                new RunMagazine(s_magazine,
                                                                                                                                .4),
                                                                                                                new RunIntake(s_intake)))))));
                bbutton.whenReleased(
                                new AngleArm(ArmAngleReturnAfterMacro, s_arm, s_vision));
                final JoystickButton startButton = new JoystickButton(driverController, 8);
                startButton.whenPressed(c_toggleCompressor);
                final JoystickButton yButton = new JoystickButton(driverController, 4);
                yButton.whenPressed(c_toggleIntake);
                SmartDashboard.putNumber("Arm Value", .1);
                final JoystickButton fenderButton = new JoystickButton(manipulatorController, 1);
                // fenderButton.whileHeld(new ParallelCommandGroup(new AngleArm(.1, s_arm), new
                // SpinShooter(s_shooter, manipulatorController, 2600)));
                fenderButton.whileHeld((new PrepareMagazineToShoot(s_magazine, s_intake)
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 1600, s_vision))
                                .raceWith(new AngleArm(.4, s_arm, s_vision))
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 1600, s_vision)
                                                .alongWith(new RunMagazine(s_magazine, .4)))));
                fenderButton.whenReleased(new AngleArm(ArmAngleReturnAfterMacro, s_arm, s_vision)); // AngleArm was .35
                final JoystickButton fenderHighButton = new JoystickButton(manipulatorController, 2);
                // fenderButton.whileHeld(new ParallelCommandGroup(new AngleArm(.1, s_arm), new
                // SpinShooter(s_shooter, manipulatorController, 2600)));
                fenderHighButton.whileHeld((new PrepareMagazineToShoot(s_magazine, s_intake)
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 3000, s_vision))
                                .raceWith(new AngleArm(.37, s_arm, s_vision))
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 3000, s_vision)
                                                .alongWith(new RunMagazine(s_magazine, .4)))));
                fenderHighButton.whenReleased(new AngleArm(ArmAngleReturnAfterMacro, s_arm, s_vision));
                final JoystickButton hooksButton = new JoystickButton(manipulatorController, 7);
                hooksButton.whenPressed(new ToggleHook(s_pneumatics));

                final JoystickButton hooksButton2 = new JoystickButton(driverController, 7);
                hooksButton2.whenPressed(new ToggleHook(s_pneumatics));

                final POVButton spinAtSDRPM = new POVButton(manipulatorController, 0);
                spinAtSDRPM.whenPressed((new PrepareMagazineToShoot(s_magazine, s_intake)
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 0, s_vision))
                                .raceWith(new AngleArmLL(s_arm, s_vision))
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 0, s_vision)
                                                .alongWith(new WaitCommand(.5).andThen(new RunMagazine(s_magazine, .4)
                                                                .alongWith(new RunIntake(s_intake)))))));
                spinAtSDRPM.whenReleased(new AngleArm(ArmAngleReturnAfterMacro, s_arm, s_vision));

                final JoystickButton resetWinchEncoder = new JoystickButton(driverController, 10);
                resetWinchEncoder.whenPressed(new ResetClimberEncoder(s_climber));

                final JoystickButton limelightToggle = new JoystickButton(driverController, 3);
                limelightToggle.whenPressed(new ToggleLights(s_vision));

                final JoystickButton tarmacHighShot = new JoystickButton(manipulatorController, 4);
                tarmacHighShot.whileHeld((new PrepareMagazineToShoot(s_magazine, s_intake)
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 3500, s_vision))
                                .raceWith(new AngleArm(.50, s_arm, s_vision))
                                .andThen(new SpinShooter(s_shooter, manipulatorController, 3500, s_vision)
                                                .alongWith(new RunMagazine(s_magazine, .4)))));
                tarmacHighShot.whenReleased(new AngleArm(ArmAngleReturnAfterMacro, s_arm, s_vision));
                SmartDashboard.putData(new RunClimberTillLimit(s_climber, -.5));

                final JoystickButton spinAtSDRPMButton = new JoystickButton(manipulatorController, 3);
                spinAtSDRPMButton.whileHeld(new SpinArmAtSDRPM(s_shooter));

                final JoystickButton toggleMagButton = new JoystickButton(manipulatorController, 9);
                toggleMagButton.toggleWhenPressed(new RunMagazine(s_magazine, 0));

                final POVButton driver0 = new POVButton(driverController, 0);
                final POVButton driver90 = new POVButton(driverController, 90);
                final POVButton driver180 = new POVButton(driverController, 180);
                final POVButton driver270 = new POVButton(driverController, 270);

                // driver90.whileHeld(new AngleArmSlow(Arm.maxPotOutput, s_arm));
                // driver270.whileHeld(new AngleArmSlow(Arm.minPotOutput, s_arm));

                driver0.whenPressed(new AngleArm(Arm.minPotOutput, s_arm, s_vision)
                                .andThen(new RunClimberTillLimit(s_climber, .5))
                                .andThen(new SetClimbPistons(Value.kForward, s_pneumatics))
                                .andThen(new WinchClimberRelative(-7, s_climber)), true);
                driver90.toggleWhenPressed(
                                new RunClimberTillLimit(s_climber, -1).andThen(new ApplyClimberPower(s_climber, -.8)
                                                .raceWith(new AngleArmFast(Constants.armPotPassValue, s_arm))
                                                .andThen(new ToggleHook(s_pneumatics).andThen(new WaitCommand(2)
                                                                .andThen(new AngleArmFast(Arm.maxPotOutput, s_arm)
                                                                                .andThen(new WinchClimberRelative(55,
                                                                                                s_climber)))))));
                final POVButton manip90 = new POVButton(manipulatorController, 90);
                manip90.toggleWhenPressed(
                                new RunClimberTillLimit(s_climber, -1).andThen(new ApplyClimberPower(s_climber, -.8)
                                                .raceWith(new AngleArmFast(Constants.armPotPassValue, s_arm))
                                                .andThen(new ToggleHook(s_pneumatics).andThen(new WaitCommand(2)
                                                                .andThen(new AngleArmFast(Arm.maxPotOutput, s_arm)
                                                                                .andThen(new WinchClimberRelative(55,
                                                                                                s_climber)))))));
                // (new WinchClimberRelative(0, s_climber).andThen(new
                // RunClimberTillLimit(s_climber, .6).alongWith(new
                // AngleArmFast(Arm.maxPotOutput, s_arm))).alongWith(new
                // SetClimbPistons(Value.kReverse, s_pneumatics))).andThen(new
                // AngleArmFast(Constants.armPotNextBar, s_arm)))), true);
                driver270.toggleWhenPressed(
                                (new ApplyClimberPower(s_climber, -1).withTimeout(.5).alongWith(
                                                new WaitCommand(0).andThen(new ToggleHook(s_pneumatics)))).andThen(
                                                                new WaitCommand(0).andThen(
                                                                                new AngleArmFast(Arm.minPotOutput,
                                                                                                s_arm))));
                driver180.whileActiveOnce(new RunClimberTillLimit(s_climber, .6)
                                .andThen(new AngleArmFast(Constants.armPotNextBar, s_arm)));

                SmartDashboard.putData(new WinchClimberRelative(-10, s_climber));

        }

        /**
         * This is a method run by {@link Robot} that schedules the autonomous command
         * selected in the dropdown.
         */
        public Command getAutonomousCommand() {
                return _autoChooser.getSelected();
        }

}
