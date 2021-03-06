package org.firstinspires.ftc.teamcode.basic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Target;
import org.firstinspires.ftc.teamcode.util.TargetType;

import java.io.IOException;

class BasicRobot {
    //Declare things
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotorEx flywheelMotor;
    DcMotor leftEncoder;
    DcMotor wobbleMotor;
    //    DcMotor middleEncoder;
    DcMotor rightEncoder;
    DcMotor intakeMotor;
    Telemetry telemetry;
    InterpLUT goalLut = new InterpLUT();
    InterpLUT powershotLut = new InterpLUT();
    static boolean isCalibrated = false;
    Servo sensor_servo;
    Servo hopperRotate;
    Servo hopperHammer;
//    Servo wobbleServo;
    RevColorSensorV3 colorSensor;

    public BasicRobot(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        leftEncoder = hardwareMap.get(DcMotor.class, "br");
//        middleEncoder = hardwareMap.get(DcMotor.class, "odom");
        rightEncoder = hardwareMap.get(DcMotor.class, "intake");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobble");


        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
        hopperRotate = hardwareMap.get(Servo.class, "hopper_rotate");
        hopperHammer = hardwareMap.get(Servo.class, "hopper_hammer");
//        wobbleServo = hardwareMap.get(Servo.class, "wobble");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
//        middleEncoder.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reset the encoders
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set the gain of the color sensor, basically its sensitivity. This can be adjusted.
        float gain = 2;
        colorSensor.setGain(gain);

        //Create the calibration tables from previously created files that will be used throughout the match.
        //This means we only have to read from the files once, increasing performance (I think?)
        createCalibrationInterplut(goalLut, TargetType.GOAL);
        createCalibrationInterplut(powershotLut, TargetType.POWERSHOT);
        sensor_servo.setPosition(Constants.COLOR_SERVO_UP);
        hopperDown();
        hammerIn();
    }


    /**
     * The basic teleOp driving math.
     * @param leftStickX The x coordinate of the left stick.
     * @param leftStickY The y coordinate of the left stick, FLIPPED.
     * @param rightStickX The x coordinate of the right stick.
     * @param sensitivity The amount of sensitivity to use, ranging from 0 to 1.
     *                    All powers are made into proportions of 1, then multiplied by this.
     */
    void drive(double leftStickX, double leftStickY, double rightStickX, double sensitivity) {
        //Use CODE and ALGORITHMS to figure out the correct powers for each wheel
        double flPower = (leftStickX + leftStickY + rightStickX);
        double frPower = (-leftStickX + leftStickY - rightStickX);
        double blPower = (-leftStickX + leftStickY + rightStickX);
        double brPower = (leftStickX + leftStickY - rightStickX);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        //Make all of them proportional to the greatest value and factor in the sensitivity.
        flPower = (flPower / maxPower) * sensitivity;
        frPower = (frPower / maxPower) * sensitivity;
        blPower = (blPower / maxPower) * sensitivity;
        brPower = (brPower / maxPower) * sensitivity;

        //Actually set them
        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }

    /**
     * This uses calibrated values to shoot. If the robot does not have calibrated values, it defaults back
     * to the value set in Constants.FLYWHEEL_CONSTANT.
     * It should be used in almost all autonomous and TeleOp opmodes.
     * @param pose The current pose of the robot.
     * @param target The target the robot is aiming at.
     * @param timer An ElapsedTime object initialized in milliseconds.
     * @param isFirst Whether or not this is the first time this is being called.
     */
    void getPowerAndShoot(Pose2d pose, Target target, ElapsedTime timer, boolean isFirst){
        double power;
        if (isCalibrated){
            power = getLaunchPower(pose, target);
        } else {
            power = Constants.FLYWHEEL_MAX_VELO;
        }
        shoot(power, timer, isFirst);
    }

    void wobbleGoalUp(){
        wobbleMotor.setTargetPosition(Constants.WOBBLE_GOAL_UP);
        wobbleMotor.setPower(.5);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void wobbleGoalDown(){
        wobbleMotor.setTargetPosition(Constants.WOBBLE_GOAL_DOWN);
        wobbleMotor.setPower(.5);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** This is the basic shoot command. It only shoots, and is used directly only in calibration,
     * though it can be called indirectly in getPowerAndShoot. It shouldn't be used directly in most opModes, use AimAndShoot instead.
     * @param power The power to shoot with, from 0 to 1.
     * @param timer An ElapsedTime object initialized in milliseconds.
     * @param isFirst Whether or not this is the first time this is being called.
     */
    void shoot(double power, ElapsedTime timer, boolean isFirst){
        if (isFirst) {
            timer.reset();
        }
        double timeSinceStart = timer.time();
        flywheelMotor.setPower(power);
//        hopperUp();
//        //This weird-ass piece of code is meant to reload the robot as fast as possible by alternating after a constant amount of milliseconds which should be tuned
//        if (timeSinceStart % Constants.DOUBLE_HAMMER_SERVO_ROTATION_TIME_MILLISECONDS > Constants.HAMMER_SERVO_ROTATION_TIME){
//            hammerOut();
//        } else {
//            hammerIn();
//        }
    }
//    void wobbleGoalUp(){
//        wobbleServo.setPosition(WOBBLE_GOAL_UP);
//    }
//    void wobbleGoalDown(){
//        wobbleServo.setPosition(WOBBLE_GOAL_DOWN);
//    }

    void driveStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    void stopFlywheel(){
        flywheelMotor.setPower(0);
    }
    void hopperDown(){
        hopperRotate.setPosition(Constants.HOPPER_DOWN);
    }
    void hopperUp(){
        hopperRotate.setPosition(Constants.HOPPER_UP);
    }
    void hammerIn(){
        hopperHammer.setPosition(Constants.HAMMER_BACK);
    }
    void hammerOut(){
        hopperHammer.setPosition(Constants.HAMMER_PUSH);
    }

    /**
     * Gets the current distance from the target. Shouldn't be called directly in most opModes,
     * but it's used by getLaunchPower(), and therefore by aimAndShoot().
     * @param pose The current pose of the robot. Get it using [localizer].getPose()
     * @param target The target the robot is aiming at.
     * @return The distance of the robot from the target in inches.
     */
    public double getDistanceFromTarget (Pose2d pose, Target target){
        double currentX = pose.getX();
        double currentY = pose.getY();
        double xDistance = Math.abs(currentX - target.getY());
        double yDistance = 120 - currentY;
        return Math.hypot(xDistance, yDistance);
    }

    /**
     * Gets the direction that the robot should aim towards to hit a specific target.
     * @param pose The current pose of the robot.
     * @param target The target the robot is aiming at.
     * @return The ideal angle of the robot, in RADIANS. For degrees use getDegreesToTarget.
     */
    public double getAngleToTarget (Pose2d pose, Target target) {
        double currentX = pose.getX();
        double currentY = pose.getY();
        double xDistance = Math.abs(currentX - target.getY());
        double yDistance = 120 - currentY;
        //If you remember your trig, the tangent of an angle is its opposite over its adjacent
        //Arctan, the inverse of tangent, gets an angle from a given opposite over adjacent.
        return Math.atan(yDistance/xDistance);
    }
    /**
     * Gets the direction that the robot should aim towards to hit a specific target.
     * Just outputs getAngleToTarget converted to degrees.
     * @param pose The current pose of the robot.
     * @param target The target the robot is aiming at.
     * @return The ideal angle of the robot, in DEGREES. For radians use getAngleToTarget.
     */
    public double getDegreesToTarget(Pose2d pose, Target target) {
        return Math.toDegrees(getAngleToTarget(pose, target));
    }

    /**
     * This gets the correct launch power for a target based off
     * an average of the two nearest files which were stored during calibration.
     * @param pose The current pose of the robot. Get it using [lcalizername].getPoseEstimate
     * @param target The target the robot is aiming at.
     * @return The correct power to use from a given point aiming at a specific target.
     */
    public double getLaunchPower(Pose2d pose, Target target) {
        //If we're shooting at a goal, get it from the goal interplut, otherwise get it from the powershot. The other one is just a sanity check.
        //Android Studio made me do it, though it should be impossible to come across.
        switch (target.getTargetType()) {
            case GOAL:
                return goalLut.get(getDistanceFromTarget(pose, target));
            case POWERSHOT:
                return powershotLut.get(getDistanceFromTarget(pose, target));
            default:
                telemetry.speak("Something's Wrong!");
                telemetry.addData("ERROR:", "No target type");
                telemetry.update();
                return 0;
        }
    }
    private void createCalibrationInterplut(InterpLUT lut, TargetType targetType){
        //Read the files we made in CalibrateGoal/Powershot, and make an interplut out of them.
        //If there are no files there, tell the rest of the class that the robot isn't calibrated.
        for (int i = Constants.MINIMUM_DISTANCE; i <= Constants.MAXIMUM_DISTANCE; i += Constants.CALIBRATION_INTERVAL) {
            try {
                lut.add(i, Double.parseDouble(ReadWriteFile.readFileOrThrow(AppUtil.getInstance().getSettingsFile(String.valueOf(targetType)+i+".txt"))));
                isCalibrated = true;
            } catch (IOException e) {
                telemetry.speak("Warning: Calibration Failed");
                telemetry.addData("Calibration","Failed");
                telemetry.update();
                isCalibrated = false;
            }
        }
    }

    public void shoot(Pose2d pose, Target target) {
        flywheelMotor.setPower(getLaunchPower(pose, target));
    }
}
