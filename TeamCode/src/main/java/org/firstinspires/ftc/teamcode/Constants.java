package org.firstinspires.ftc.teamcode;


class Constants {
    // DRIVING CONSTANTS
    public static final float DRIVE_POWER = 1;
    public static final float DRIVE_POWER_SLOW = .4f;
    public static final float DRIVE_STICK_THRESHOLD = .3f;
    public static final float TRIGGER_THRESHOLD = .65f;
    // ODOMETRY CONSTANTS
    public static final float ODOMETRY_WHEEL_DIAMETER = 6.18f; //in inches
    public static final float COUNTS_PER_ROTATION = 200; //because of the encoders we have
    public static final double DISTANCE_PER_COUNT = ODOMETRY_WHEEL_DIAMETER/COUNTS_PER_ROTATION;
    public static final double COUNTS_PER_INCH = 1/DISTANCE_PER_COUNT;

    // TARGET CONSTANTS
    // These need to be changed
    public static final Target powershot1 = new Target(30,TargetType.POWERSHOT);
    public static final Target powershot2 = new Target(36,TargetType.POWERSHOT);
    public static final Target powershot3 = new Target(42,TargetType.POWERSHOT);
    public static final Target topGoal = new Target(60, TargetType.GOAL);
}
