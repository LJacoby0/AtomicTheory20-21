package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.util.Target;
import org.firstinspires.ftc.teamcode.util.TargetType;

public class Constants {
    // DRIVING CONSTANTS
    public static final float DRIVE_POWER = .9f;
    public static final float DRIVE_POWER_SLOW = .4f;
    public static final float DRIVE_STICK_THRESHOLD = .2f;
    public static final float TRIGGER_THRESHOLD = .65f;

    //FLYWHEEL CONSTANTS
    public static final int FLYWHEEL_MAX_VELO = 2000;

    //TURRET CONSTANTS
    public static final double TICKS_PER_ROTATION = 5281.1;
    public static final double RADIANS_PER_MOTOR_ROTATION = 0.40105438;
    public static final double DEGREES_PER_MOTOR_ROTATION = Math.toDegrees(RADIANS_PER_MOTOR_ROTATION);
    public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION/DEGREES_PER_MOTOR_ROTATION;
    public static final int MOTOR_RPM = 30;
    public static final double TURRET_ROTATION_SPEED = MOTOR_RPM * DEGREES_PER_MOTOR_ROTATION / 60; //In degrees per second
    public static final double TURRET_MAX_ROTATiON_DEGREES = 30;
    public static final double TURRET_MIN_ROTATiON_DEGREES = 0;





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
    public static final int HAMMER_SERVO_ROTATION_TIME = 450;
    public static final int DOUBLE_HAMMER_SERVO_ROTATION_TIME_MILLISECONDS = 2 * HAMMER_SERVO_ROTATION_TIME;
    public static final double HAMMER_PUSH = .68;
    public static final double HAMMER_BACK = .90;
    public static final double HAMMER_VERY_IN = .98;
    public static final double HOPPER_UP = .6;
    public static final double HOPPER_DOWN = .77;
    public static final double COLOR_SERVO_DOWN = .17;
    public static final double COLOR_SERVO_UP = .11;
    public static final int WOBBLE_GOAL_UP = 0;
    public static final int WOBBLE_GOAL_DOWN = 1025;


    public static final double THRESHOLD = 25;
}
