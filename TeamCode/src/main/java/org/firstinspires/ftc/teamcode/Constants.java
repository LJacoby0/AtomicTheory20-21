package org.firstinspires.ftc.teamcode;


public class Constants {
    // DRIVING CONSTANTS
    public static final float DRIVE_POWER = .9f;
    public static final float DRIVE_POWER_SLOW = .4f;
    public static final float DRIVE_STICK_THRESHOLD = .2f;
    public static final float TRIGGER_THRESHOLD = .65f;

    //FLYWHEEL CONSTANTS
    public static final int FLYWHEEL_MAX_VELO = 2000;


    // ODOMETRY CONSTANTS
    public static final float ODOMETRY_WHEEL_DIAMETER = 6.18f; //in inches
    public static final float COUNTS_PER_ROTATION = 800; //because of the encoders we have
    public static final double DISTANCE_PER_COUNT = ODOMETRY_WHEEL_DIAMETER/COUNTS_PER_ROTATION;
    public static final double COUNTS_PER_INCH = 1/DISTANCE_PER_COUNT;

    // TARGET CONSTANTS
    // These need to be changed
    public static final Target redPowershot1 = new Target(30, TargetType.POWERSHOT);
    public static final Target redPowershot2 = new Target(36, TargetType.POWERSHOT);
    public static final Target redPowershot3 = new Target(42, TargetType.POWERSHOT);
    public static final Target redTopGoal = new Target(60, TargetType.GOAL);
    public static final Target bluePowershot1 = new Target(29, TargetType.POWERSHOT);
    public static final Target bluePowershot2 = new Target(38, TargetType.POWERSHOT);
    public static final Target bluePowershot3 = new Target(45, TargetType.POWERSHOT);
    public static final Target blueTopGoal = new Target(60, TargetType.GOAL);

    // CALIBRATION CONSTANTS
    public static final int MINIMUM_DISTANCE = 48;
    public static final int MAXIMUM_DISTANCE = 120;
    public static final int CALIBRATION_INTERVAL = 12;

    // SERVO CONSTANTS
    //The rotation times are definitely less than I have them as here, tuning is needed
    public static final int HAMMER_SERVO_ROTATION_TIME = 600;
    public static final int DOUBLE_HAMMER_SERVO_ROTATION_TIME_MILLISECONDS = 2*HAMMER_SERVO_ROTATION_TIME;
    public static final int HOPPER_SERVO_ROTATION_TIME_MILLISECONDS = 1000;
    public static final double HAMMER_OUT = .68;
    public static final double HAMMER_IN = .90;
    public static final double HAMMER_VERY_IN = .98;
    public static final double HOPPER_UP = .6;
    public static final double HOPPER_DOWN = .77;
    public static final double COLOR_SERVO_DOWN = .17;
    public static final double COLOR_SERVO_UP = .11;
    public static final int WOBBLE_GOAL_UP = 0;
    public static final int WOBBLE_GOAL_DOWN = 1025;





}
