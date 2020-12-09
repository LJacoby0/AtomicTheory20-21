package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

class Robot {
    //Declare motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    //DcMotor flywheelMotor;
    DcMotor leftEncoder;
    DcMotor middleEncoder;
    DcMotor rightEncoder;
    Telemetry telemetry;
    InterpLUT goalLut = new InterpLUT();
    InterpLUT powershotLut = new InterpLUT();
    boolean isCalibrated = true;

    public Robot(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        leftEncoder = hardwareMap.get(DcMotor.class, "odol");
        middleEncoder = hardwareMap.get(DcMotor.class, "odom");
        rightEncoder = hardwareMap.get(DcMotor.class, "odor");

        //flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        createCalibrationInterplut(goalLut, TargetType.GOAL);
        createCalibrationInterplut(powershotLut, TargetType.POWERSHOT);
    }


    
    void drive(double leftStickX, double leftStickY, double rightStickX, double sensitivity) {
        //Use math to figure out the correct powers for each wheel
        double flPower = (leftStickX + leftStickY + rightStickX);
        double frPower = (-leftStickX + leftStickY - rightStickX);
        double blPower = (-leftStickX + leftStickY + rightStickX);
        double brPower = (leftStickX + leftStickY - rightStickX);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        //If the maxPower is less than 1, make it 1
        maxPower = Math.max(maxPower, 1);

        //Make all of them proportional to the greatest value and factor in slow mode.
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
    void driveStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    static double startTime;

    //However, when it's the first time, we call this one
    void aimAndShoot(Pose2d pose, Target target, ElapsedTime timer, boolean isFirst){
        double power;
        if (isCalibrated){
            if (target.getTargetType() == TargetType.GOAL){
                power = goalLut.get(getDistanceFromTarget(pose, target));
            } else {
                power = powershotLut.get(getDistanceFromTarget(pose, target));
            }
        } else {
            power = Constants.POWER_CONSTANT;
        }
        shoot(power, timer, isFirst);
    }
    //This is the standard shoot command
    void shoot(double power, ElapsedTime timer, boolean isFirst){
        if (isFirst) {
            startTime = timer.time();
        }
        //flywheelMotor.setPower(power);
        double timeSinceStart = timer.time() - startTime;
        if (timeSinceStart>500 && timeSinceStart % 500<250){
            //loadServo.setPosition(Constants.back);
        } else {
            //loadServo.setPosition(Constants.load);
        }
    }

    //Gets the distance from the target with Math.hypot between the robot and the target positional constants
    public double getDistanceFromTarget (Pose2d pose, Target target){
        double currentX = pose.getX();
        double currentY = pose.getY();
        double xDistance = Math.abs(currentX - target.getX());
        double yDistance = 120 - currentY;
        return Math.hypot(xDistance, yDistance);
    }
    // This gets the correct launch angle for a target based off an average of the two nearest files which were stored during calibration.
    public double getLaunchPower(@NotNull Target target, Pose2d pose) {
        //If we're shooting at a goal, get it from the goal interplut, otherwise get it from the powershot. The other one is just a sanity check.
        if (target.getTargetType() == TargetType.GOAL){
            return goalLut.get(getDistanceFromTarget(pose, target));
        } else if(target.getTargetType() == TargetType.POWERSHOT){
            return powershotLut.get(getDistanceFromTarget(pose, target));
        }
        else {
            telemetry.speak("Something's Wrong!");
            telemetry.addData("ERROR:","No target type");
            telemetry.update();
            return 1;
        }
    }
    private void createCalibrationInterplut(InterpLUT lut, TargetType targetType){
        //Read the files we made in CalibrateGoal/Powershot, and make an interplut out of them.
        for (int i = 60; i <= 120; i+=10) {
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
}
