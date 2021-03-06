

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Target;
import org.firstinspires.ftc.teamcode.util.TargetType;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;

public class Robot {
    //Declare things
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx flywheelMotor;
    public DcMotorEx turretMotor;
    public DcMotor leftEncoder;
    public DcMotor middleEncoder;
    public DcMotor rightEncoder;
    public DcMotor intakeMotor;
    public DcMotor wobbleMotor;
    Telemetry telemetry;
    InterpLUT goalLut = new InterpLUT();
    InterpLUT powershotLut = new InterpLUT();
    static boolean isCalibrated = false;
    public Servo sensor_servo;
    public Servo hopperRotate;
    public Servo hopperHammer;
    public ColorRangeSensor colorSensor;

    public static final int VELO_TOLERANCE = 50;

    public Robot(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        leftEncoder = hardwareMap.get(DcMotor.class, "odol");
        middleEncoder = hardwareMap.get(DcMotor.class, "br");
        rightEncoder = hardwareMap.get(DcMotor.class, "intake");

        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
        hopperRotate = hardwareMap.get(Servo.class, "hopper_rotate");
        hopperHammer = hardwareMap.get(Servo.class, "hopper_hammer");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobble");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");



        // Set motor directions
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
//        middleEncoder.setDirection(DcMotor.Direction.REVERSE);
//        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to brake when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Reset the encoders
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        hammerBack();
    }

    public void rotateTurretDegrees(double degrees){
        int ticksToRotate = (int) Math.round(-1 * (degrees * Constants.TICKS_PER_DEGREE));
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + ticksToRotate);
    }
    /**
     * The basic teleOp driving math.
     * @param leftStickX The x coordinate of the left stick.
     * @param leftStickY The y coordinate of the left stick, FLIPPED.
     * @param rightStickX The x coordinate of the right stick.
     * @param sensitivity The amount of sensitivity to use, ranging from 0 to 1.
     *                    All powers are made into proportions of 1, then multiplied by this.
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX, double sensitivity) {
        //Use CODE and ALGORITHMS to figure out the correct powers for each wheel
        double flPower = (leftStickX + leftStickY + rightStickX);
        double frPower = (-leftStickX + leftStickY - rightStickX);
        double blPower = (-leftStickX + leftStickY + rightStickX);
        double brPower = (leftStickX + leftStickY - rightStickX);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        //If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

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
    public void getPowerAndShoot(Pose2d pose, Target target, ElapsedTime timer, boolean isFirst){
        double velocity;
        if (target.getTargetType().equals(TargetType.GOAL)){
            velocity = 1800;
        } else if (target.getTargetType().equals(TargetType.POWERSHOT)){
            velocity = 1625;
        } else {
            velocity = 1800;
        }
//        if (isCalibrated){
//            velocity = getLaunchPower(pose, target);
//        } else {
//            velocity = Constants.FLYWHEEL_MAX_VELO;
//        }
        shoot(velocity, timer, isFirst);
    }

    /** This is the basic shoot command. It only shoots, and is used directly only in calibration,
     * though it can be called indirectly in getPowerAndShoot. It shouldn't be used directly in most opModes, use getPowerAndShoot instead.
     * @param velocity The velocity to shoot with, in encoder ticks per second.
     * @param timer An ElapsedTime object initialized in milliseconds.
     * @param isFirst Whether or not this is the first time this is being called.
     */
    public void shoot(double velocity, ElapsedTime timer, boolean isFirst){
        if (isFirst) {
            timer.reset();
        }
        double timeSinceStart = timer.time();
        flywheelMotor.setVelocity(velocity);
        hopperUp();
        boolean isVelocityCorrect = flywheelMotor.getVelocity() > velocity - VELO_TOLERANCE && flywheelMotor.getVelocity() < velocity + VELO_TOLERANCE;
        //This weird-ass piece of code is meant to reload the robot as fast as possible by alternating after a constant amount of milliseconds which should be tuned
        boolean isSwitchTime = timeSinceStart % Constants.DOUBLE_HAMMER_SERVO_ROTATION_TIME_MILLISECONDS > Constants.HAMMER_SERVO_ROTATION_TIME;
//        boolean isHopperUp = timeSinceStart > Constants.HOPPER_SERVO_ROTATION_TIME_MILLISECONDS;
        if (isSwitchTime && isVelocityCorrect/* && isHopperUp*/){
            hammerPush();
        } else {
            hammerBack();
        }
    }
    public void wobbleGoalUp(){
        wobbleMotor.setTargetPosition(Constants.WOBBLE_GOAL_UP);
        wobbleMotor.setPower(.2);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void wobbleGoalDown(){
        wobbleMotor.setTargetPosition(Constants.WOBBLE_GOAL_DOWN);
        wobbleMotor.setPower(.5);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void stopFlywheel(){
        flywheelMotor.setPower(0);
    }
    public void hopperDown(){
        hopperRotate.setPosition(Constants.HOPPER_DOWN);
    }
    public void hopperUp(){
        hopperRotate.setPosition(Constants.HOPPER_UP);
    }
    public void hammerBack(){
        hopperHammer.setPosition(Constants.HAMMER_BACK);
    }
    public void hammerPush(){
        hopperHammer.setPosition(Constants.HAMMER_PUSH);
    }

    /**
     * Gets the current distance from the target. Shouldn't be called directly in most opModes,
     * but it's used by getLaunchPower(), and therefore by aimAndShoot().
     * Depreciated as distance from target now uses computer vision.
     * @param pose The current pose of the robot. Get it using [localizer].getPose()
     * @param target The target the robot is aiming at.
     * @return The distance of the robot from the target in inches.
     */
    @Deprecated
    public double getDistanceFromTarget (Pose2d pose, Target target){
        double currentX = pose.getX();
        double currentY = pose.getY();
        double xDistance = Math.abs(currentY - target.getY());
        double yDistance = 60 - currentX;
        return Math.hypot(xDistance, yDistance);
    }

    /**
     * Gets the direction that the robot should aim towards to hit a specific target. This one isn't used yet.
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
     * @param distance The current distance from the target, get it using
     * @param isPowershot The target the robot is aiming at.
     * @return The correct velocity to use from a given point aiming at a specific target.
     */
    public double getLaunchPower(double distance, boolean isPowershot) {
        //If we're shooting at a goal, get it from the goal interplut, otherwise get it from the powershot. The other one is just a sanity check.
        //Android Studio made me do it, though it should be impossible to come across.
        if (isPowershot) {
                return powershotLut.get(distance);
            } else {
                return goalLut.get(distance);
        }
    }
    private void createCalibrationInterplut(InterpLUT lut, TargetType targetType){
        //Read the files we made in CalibrateGoal/Powershot, and make an interplut out of them.
        //If there are no files there, tell the rest of the class that the robot isn't calibrated.
        try {
            for (int i = Constants.MINIMUM_DISTANCE; i <= Constants.MAXIMUM_DISTANCE; i += Constants.CALIBRATION_INTERVAL) {

                lut.add(i, Integer.parseInt(ReadWriteFile.readFileOrThrow(AppUtil.getInstance().getSettingsFile(String.valueOf(targetType) + i + ".txt"))));

            }
            lut.createLUT();
            telemetry.addData("Calibration","Success");
            telemetry.update();
            isCalibrated = true;
        } catch (IOException e) {
            telemetry.speak("Warning: Calibration Failed");
            telemetry.addData("Calibration","Failed");
            telemetry.update();
            isCalibrated = false;
        }
    }
    //Hopefully this works, it's meant to be put into a while loop as the method and expression
    //timer must be reset before using
    public boolean shootRing(@NotNull ElapsedTime timer, double power) {
        double targetVelocity = power/*getLaunchPower(pose, target)*/;
        flywheelMotor.setVelocity(targetVelocity);
        double timeSinceStart = timer.time();
        hopperUp();
        boolean isVelocityCorrect = flywheelMotor.getVelocity() > targetVelocity - VELO_TOLERANCE && flywheelMotor.getVelocity() < targetVelocity + VELO_TOLERANCE;
        if (isVelocityCorrect){
            hammerPush();
            return true;
        } else {
            hammerBack();
            return false;
        }
    }
}
