package org.firstinspires.ftc.teamcode;


class Constants {
    // DRIVING CONSTANTS
    public static final float DRIVE_POWER = 1;
    public static final float DRIVE_POWER_SLOW = .4f;
    public static final float DRIVE_STICK_THRESHOLD = .3f;
    public static final float TRIGGER_THRESHOLD = .65f;
    public static final float FLYWHEEL_CONSTANT = .5f;


    // ODOMETRY CONSTANTS
    public static final float ODOMETRY_WHEEL_DIAMETER = 6.18f; //in inches
    public static final float COUNTS_PER_ROTATION = 200; //because of the encoders we have
    public static final double DISTANCE_PER_COUNT = ODOMETRY_WHEEL_DIAMETER/COUNTS_PER_ROTATION;
    public static final double COUNTS_PER_INCH = 1/DISTANCE_PER_COUNT;

    // TARGET CONSTANTS
    // These need to be changed
    public static final Target redPowershot1 = new Target(30,TargetType.POWERSHOT);
    public static final Target redPowershot2 = new Target(36,TargetType.POWERSHOT);
    public static final Target redPowershot3 = new Target(42,TargetType.POWERSHOT);
    public static final Target redTopGoal = new Target(60, TargetType.GOAL);
    public static final Target bluePowershot1 = new Target(30,TargetType.POWERSHOT);
    public static final Target bluePowershot2 = new Target(36,TargetType.POWERSHOT);
    public static final Target bluePowershot3 = new Target(42,TargetType.POWERSHOT);
    public static final Target blueTopGoal = new Target(60, TargetType.GOAL);

    // DISTANCE CONSTANTS
    public static final int MINIMUM_DISTANCE = 60;
    public static final int MAXIMUM_DISTANCE = 140;

    // SERVO CONSTANTS
    public static final int SERVO_ROTATION_TIME_MILLISECONDS = 700;
    //This is just a best practice, it could also just be put in the one file we need it in (AdvancedTeleOp)
    public static final int HALF_SERVO_ROTATION_TIME = SERVO_ROTATION_TIME_MILLISECONDS / 2;
}
