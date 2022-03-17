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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.TrajectoryHelper;
import frc.robot.commands.AngleArm;
import frc.robot.commands.ArmManual;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseMagazine;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunMagazine;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleHook;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.TrackBall;
import frc.robot.commands.WinchClimber;
import frc.robot.commands.WinchToSetpoint;
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
}

/**
* Create joysticks objects. We use xbox controllers.
* These will be used for button bindings in the configureButtonBindings()
* method is called.
*/
public XboxController driverController = new XboxController(0);
public XboxController manipulatorController = new XboxController(1);

/**
* Define instances of the commands. These are static, so only one instance ever
* exists.
* They are final, so they cannot be overwritten, and they are public,
* so we can access them from helper classes.
*/
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
private final RunIntake c_runIntake = new RunIntake(s_intake);
private final ReverseIntake c_reverseIntake = new ReverseIntake(s_intake);
// private final SpinShooter c_spinShooter = new SpinShooter(s_shooter,
// driverController, 3000);
private final ToggleCompressor c_toggleCompressor = new ToggleCompressor(s_pneumatics);
private final ToggleIntake c_toggleIntake = new ToggleIntake(s_pneumatics);
private final WinchClimber c_winchClimber = new WinchClimber(s_climber, driverController);
private final Command c_fenderShot = new SequentialCommandGroup(new AngleArm(.1, s_arm),
new ReverseMagazine(s_magazine).withTimeout(.5),
new RunMagazine(s_magazine, .6).alongWith().withTimeout(2))
.alongWith(new SpinShooter(s_shooter, driverController, 2600).withTimeout(2.7));
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
TrajectoryHelper.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),List.of(new Translation2d(1, 0.1),new Translation2d(2,1)),new Pose2d(5, 0, new Rotation2d(0)),false, 3, 2, 1, 0, 0, 7))
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

// _autoChooser.addOption("2 ball low goal",
// new ResetOdometry(new Pose2d(0, 0, new Rotation2d(0)), s_drivetrain).andThen(
// new ToggleIntake(s_pneumatics).andThen(
// new ParallelCommandGroup(
// new AngleArm(.32, s_arm),
// new SpinShooter(s_shooter,
// manipulatorController,
// 1600),
// new RunMagazine(s_magazine, .6))
// .withTimeout(2)
// .andThen(new RunIntake(
// s_intake).raceWith(
// TrajectoryHelper.createTrajectoryCommand(
// TrajectoryHelper.generateTrajectory(
// new Pose2d(0, 0, new Rotation2d(
// 0)),
// List.of(new Translation2d(
// 1,
// 0)),
// new Pose2d(2, -1,
// new Rotation2d(0)),
// false,
// 2,
// 1,
// 1,
// 0,
// 0,
// 7)))
// .andThen(
// new AngleArm(.1, s_arm)
// .andThen(
// TrajectoryHelper.createTrajectoryCommand(
// TrajectoryHelper.generateTrajectory(
// new Pose2d(2, -1,
// new Rotation2d(0)),
// List.of(new Translation2d(
// 1,
// 0)),
// new Pose2d(0, 0,
// new Rotation2d(0)),
// true,
// 2,
// 1,
// 1,
// 0,
// 0,
// 7))))
// .andThen(
// new ParallelCommandGroup(
// new AngleArm(.32,
// s_arm),
// new SpinShooter(s_shooter,
// manipulatorController,
// 1600),
// new RunMagazine(s_magazine,
// .6),
// new RunIntake(s_intake))
// .withTimeout(2))))));
_autoChooser.addOption("3 ball high goal right tarmac",
new ToggleIntake(s_pneumatics).andThen(
new ResetOdometry(new Pose2d(7.74, 2.81, Rotation2d.fromDegrees(-110.64)), s_drivetrain)
.andThen(
new ParallelCommandGroup(new AngleArm(.17, s_arm),
new SpinShooter(s_shooter,
manipulatorController,
3000),
new RunMagazine(s_magazine, .6))
.withTimeout(1.5)
.andThen(
new AngleArm(.07,
s_arm).andThen(
        new RunIntake(s_intake)
.alongWith(new RunMagazine(
s_magazine,
.3)).raceWith(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(7.74, 2.81,
Rotation2d.fromDegrees(
-110.64)),
List.of(new Translation2d(
6,
2.),
new Translation2d(
4.71,
1.25),
new Translation2d(
5.92,
1)),
new Pose2d(8, .52,
Rotation2d.fromDegrees(
-90)),
false,
3,
2,
1,
0,
0,
7))
)
.andThen(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(8, .52,
Rotation2d.fromDegrees(
-90)),
List.of(),
new Pose2d(7.74, 2.81, Rotation2d.fromDegrees(-110.64)),
true,
3,
2,
1,
0,
1,
7))
.andThen(
new RunMagazine(
s_magazine,
-.6)
.withTimeout(
.4)
.andThen(new AngleArm(.17, s_arm).andThen(
new ParallelCommandGroup(
new RunIntake(
s_intake),
new RunMagazine(
s_magazine,
.6)).alongWith(
new SpinShooter(
s_shooter,
manipulatorController,
3000)))))))))));

_autoChooser.addOption("3 ball high goal left tarmac",
(
new ToggleIntake(s_pneumatics).andThen(
new ResetOdometry(new Pose2d(6.96, 4.54, Rotation2d.fromDegrees(159.1)), s_drivetrain)
.andThen(
new ParallelCommandGroup(new AngleArm(.17, s_arm),
new SpinShooter(s_shooter,
manipulatorController,
3000),
new RunMagazine(s_magazine, .6))
.withTimeout(.75)
.andThen(
        new RunIntake(s_intake)
.alongWith(new RunMagazine(
s_magazine,
.3)).raceWith(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(6.96, 4.54,
Rotation2d.fromDegrees(
159.1)),
List.of(new Translation2d(
3.72,
4.13)),
new Pose2d(1.58, .68,
Rotation2d.fromDegrees(
-135.34)),
false,
3,
2,
1,
0,
0,
7))
)
.andThen(new RunIntake(
        s_intake).alongWith(
        new RunMagazine(s_magazine,
        .4)).withTimeout(1).andThen(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
        new Pose2d(1.58, .68,
Rotation2d.fromDegrees(
-135.34)),
List.of(new Translation2d(
3.69,
4.01)),
new Pose2d(6.96, 4.54,
Rotation2d.fromDegrees(
159.1)),
true,
3,
2,
1,
0,
1,
7)))
.andThen(
new RunMagazine(s_magazine,
-.6)
.withTimeout(.5)
.andThen(
new ParallelCommandGroup(
new AngleArm(
.17,
s_arm),
new SpinShooter(
s_shooter,
manipulatorController,
3000),
new RunMagazine(
s_magazine,
.6).alongWith(new RunIntake(s_intake))).withTimeout(
1.5)))))))));

_autoChooser.addOption("Two ball high goal left tarmac",
new ToggleIntake(s_pneumatics).andThen(
new ResetOdometry(new Pose2d(6.94, 4.58, Rotation2d.fromDegrees(158.73)), s_drivetrain)
.andThen(new RunIntake(s_intake).raceWith(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(6.94, 4.58,
Rotation2d.fromDegrees(
158.73)),
List.of(),
new Pose2d(5, 6.4,
Rotation2d.fromDegrees(
135.81)),
false, 3, 2, 1, 0, 0,
7))
)
.andThen(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(5, 6.4,
Rotation2d.fromDegrees(
135.81)),
List.of(),
new Pose2d(6.94, 4.58,
Rotation2d.fromDegrees(
158.73)),
true,
2,
1,
1,
0,
1,
7))
.andThen(new RunMagazine(
s_magazine,
-.6).withTimeout(
.5)
.andThen(new ParallelCommandGroup(
new AngleArm(.17,
s_arm),
new SpinShooter(s_shooter,
manipulatorController,
3000),
new RunMagazine(s_magazine,
.6), new RunIntake(s_intake))))))));

_autoChooser.addOption("Two ball right tarmac right ball",
new ToggleIntake(s_pneumatics).andThen(
new ResetOdometry(new Pose2d(7.80, 2.77, Rotation2d.fromDegrees(-108.24)),
s_drivetrain).andThen(
        (new RunIntake(s_intake).alongWith(new RunMagazine(
                s_magazine,
                .3))).raceWith(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(7.80, 2.77,
Rotation2d.fromDegrees(
-108.24)),
List.of(),
new Pose2d(8.25, 0.7,
Rotation2d.fromDegrees(
-92.94)),
false, 3, 2, 1, 0, 0,
7)))
))
.andThen(TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(7.6, 0.73,
Rotation2d.fromDegrees(
-92.94)),
List.of(),
new Pose2d(7.80, 2.77,
Rotation2d.fromDegrees(
-108.24)),
true, 2, 1, 1, 0, 1,
7)))
.andThen(new RunMagazine(s_magazine, -.6)
.withTimeout(.5)
.andThen(new ParallelCommandGroup(
new AngleArm(.17,
s_arm),
new SpinShooter(s_shooter,
manipulatorController,
3000),
new RunMagazine(s_magazine,
.6), new RunIntake(s_intake)).withTimeout(
1.5))));

_autoChooser.addOption("Two ball right tarmac left ball",
new ToggleIntake(s_pneumatics).andThen(
new ResetOdometry(new Pose2d(7.74, 2.83, Rotation2d.fromDegrees(-111.23)),
s_drivetrain).andThen(
        new RunIntake(s_intake)
.alongWith(new RunMagazine(
s_magazine,
.3)).raceWith(
TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(7.74, 2.83,
Rotation2d.fromDegrees(
-111.23)),
List.of(),
new Pose2d(4.71, 1.5,
Rotation2d.fromDegrees(
-164.74)),
false, 3, 2, 2, 0, 0,
7))
))
.andThen(TrajectoryHelper.createTrajectoryCommand(
TrajectoryHelper.generateTrajectory(
new Pose2d(4.71, 1.5,
Rotation2d.fromDegrees(
-164.74)),
List.of(),
new Pose2d(7.74, 2.83,
Rotation2d.fromDegrees(
-111.23)),
true, 3, 2, 1, 0, 1,
7)))
.andThen(new RunMagazine(s_magazine, -.6).withTimeout(.25).andThen(new ParallelCommandGroup(new AngleArm(.17,s_arm),new SpinShooter(s_shooter,manipulatorController,3000),new RunMagazine(s_magazine,.3)).withTimeout(1)))));

_autoChooser.addOption("Four ball from Right tarmac", 
new AngleArm(.17, s_arm).andThen(
new ToggleIntake(s_pneumatics)).andThen(
new ResetOdometry(new Pose2d(7.74, 2.81,
Rotation2d.fromDegrees(
-110.64)), s_drivetrain).andThen(
        new RunIntake(s_intake).raceWith(        
TrajectoryHelper.createTrajectoryCommand(
                TrajectoryHelper.generateTrajectory(new Pose2d(7.74, 2.81,
                Rotation2d.fromDegrees(
                -110.64)), List.of(), new Pose2d(5.5, 1.8, Rotation2d.fromDegrees(180)), false, 8, 5, 3, 0, 0, 7)
        ))).andThen(((new RunIntake(s_intake).alongWith(new RunMagazine(s_magazine, .1))).raceWith(new WaitCommand(1)).andThen(
                new RunMagazine(s_magazine, -.6).withTimeout(.2))).alongWith(
                TrajectoryHelper.createTrajectoryCommand(
                        TrajectoryHelper.generateTrajectory(new Pose2d(5.5, 1.8, Rotation2d.fromDegrees(90)), List.of(), new Pose2d(7.80, 2.77,
                        Rotation2d.fromDegrees(
                        -108.24)), true, 8, 5, 3, 0, 1, 7)
                        
                ).andThen(new ParallelCommandGroup(new AngleArm(.17,s_arm),new RunIntake(s_intake),new SpinShooter(s_shooter,manipulatorController,3000),new RunMagazine(s_magazine,.6)).withTimeout(1)))   
        ).andThen(
               
                        (new RunIntake(s_intake).alongWith(new RunMagazine(s_magazine, .3))).raceWith(
                TrajectoryHelper.createTrajectoryCommand(
                        TrajectoryHelper.generateTrajectory(new Pose2d(7.74, 2.81,
                        Rotation2d.fromDegrees(
                        -110.64)), List.of(), 
                        new Pose2d(1.64, .2,
                        Rotation2d.fromDegrees(
                        -115.34)), false, 5, 3, 2, 0, 0, 7)
                ))).andThen(
                        (new RunIntake(s_intake).alongWith(new AngleArm(.17,s_arm).alongWith(new RunMagazine(s_magazine, .6)))).withTimeout(.5).andThen(new RunMagazine(s_magazine, -.6).withTimeout(.2).andThen(new WaitCommand(15))).raceWith(
                        TrajectoryHelper.createTrajectoryCommand(
                        TrajectoryHelper.generateTrajectory(
                                new Pose2d(1.64, .2,
                                Rotation2d.fromDegrees(
                                -115.34)), List.of(), new Pose2d(7.74, 2.81,
                                Rotation2d.fromDegrees(
                                -110.64)), true, 8, 5, 3, 0, 1, 7)
                )).andThen((new RunIntake(s_intake)).raceWith(
                new ParallelCommandGroup(new AngleArm(.17,s_arm),new RunIntake(s_intake),new SpinShooter(s_shooter,manipulatorController,3000),new RunMagazine(s_magazine,.4)).withTimeout(5)))   
        ))
);



SmartDashboard.putData("Auto Mode", _autoChooser);
}

/**
* This is where we set button bindings.
* See button numbers at
* http://www.team358.org/files/programming/ControlSystem2015-2019/images/XBoxControlMapping.jpg
*/
private void configureButtonBindings() {
final JoystickButton rightBumper = new JoystickButton(driverController, 5);
rightBumper.whileHeld(c_reverseIntake);
final JoystickButton leftBumper = new JoystickButton(driverController, 6);
leftBumper.whileHeld(c_runIntake);

final JoystickButton rightBumperM = new JoystickButton(manipulatorController, 5);
rightBumperM.whileHeld(new ReverseMagazine(s_magazine).alongWith(new ReverseIntake(s_intake)));
final JoystickButton leftBumperM = new JoystickButton(manipulatorController, 6);
leftBumperM.whileHeld(new RunMagazine(s_magazine, .6).alongWith(new RunIntake(s_intake)));

final JoystickButton aButton = new JoystickButton(driverController, 1);
aButton.whileHeld(new TrackBall(s_drivetrain, true, driverController));
final JoystickButton bbutton = new JoystickButton(driverController, 2);
bbutton.whileHeld(new LimelightAim(s_drivetrain, s_vision));
final JoystickButton startButton = new JoystickButton(driverController, 8);
startButton.whenPressed(c_toggleCompressor);
final JoystickButton yButton = new JoystickButton(driverController, 4);
yButton.whenPressed(c_toggleIntake);
SmartDashboard.putNumber("Arm Value", .1);
final JoystickButton fenderButton = new JoystickButton(manipulatorController, 1);
// fenderButton.whileHeld(new ParallelCommandGroup(new AngleArm(.1, s_arm), new
// SpinShooter(s_shooter, manipulatorController, 2600)));
fenderButton.whileHeld(
new ParallelCommandGroup(new AngleArm(.32, s_arm),
new SpinShooter(s_shooter, manipulatorController, 1600)));

final JoystickButton fenderHighButton = new JoystickButton(manipulatorController, 2);
// fenderButton.whileHeld(new ParallelCommandGroup(new AngleArm(.1, s_arm), new
// SpinShooter(s_shooter, manipulatorController, 2600)));
fenderHighButton.whileHeld(
new ParallelCommandGroup(new AngleArm(.17, s_arm),
new SpinShooter(s_shooter, manipulatorController, 3000)));

final JoystickButton hooksButton = new JoystickButton(driverController, 7);
hooksButton.whenPressed(new ToggleHook(s_pneumatics));

final POVButton spinAtSDRPM = new POVButton(manipulatorController, 0);
spinAtSDRPM.whileHeld(new SpinShooter(s_shooter, manipulatorController, 0));

final POVButton winchDown = new POVButton(driverController, 180);
final POVButton winchUp = new POVButton(driverController, 0);
winchDown.whileHeld(new WinchToSetpoint(0, s_climber));
winchUp.whileHeld(new WinchToSetpoint(130, s_climber));
final POVButton cannonOutButton = new POVButton(driverController, 90);
cannonOutButton.whileHeld(new AngleArm(.36, s_arm));
final POVButton cannonInButton = new POVButton(driverController, 270);
cannonInButton.whileHeld(new AngleArm(.04, s_arm));
}      

/**
* This is a method run by {@link Robot} that schedules the autonomous command
* selected in the dropdown.
*/
public Command getAutonomousCommand() {
return _autoChooser.getSelected();
}

}
