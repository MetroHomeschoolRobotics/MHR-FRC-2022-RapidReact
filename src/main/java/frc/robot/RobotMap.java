package frc.robot;
//This file is used to store which port all motors and sensors are plugged in to. Please update after re-wiring or changing CAN ids
public class RobotMap {
    //CAN ids
    //drivetrain motor controllers
    public static int leftFrontMotor = 1;
    public static int leftRearMotor = 2;
    public static int rightFrontMotor = 3;
    public static int rightRearMotor = 4;
    //Shooter motors
    public static int leftShooterMotor = 5;
    public static int rightShooterMotor = 6;
    //winches
    public static int arm_winch = 2;
    public static int climber_winch = 1;
    //intake & indexing
    public static int intakeMotorPort = 5;
    public static int magazineMotor = 3;
    public static int indexerMotor = 4;

    //Pneumatics
    public static int intakeSolenoid1 = 0;
    public static int intakeSolenoid2 = 1;
    public static int hookSolenoid1 = 2;
    public static int hookSolenoid2 = 3;


}