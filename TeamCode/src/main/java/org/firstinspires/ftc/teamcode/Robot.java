package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

class Robot {
    //Declare motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Telemetry telemetry;
    public Robot(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




    /**
     *
     * @param leftStickX
     * @param leftStickY
     * @param rightStickX
     * @param sensitivity
     */
    void drive(double leftStickX, double leftStickY, double rightStickX, double sensitivity) {
        //Use math to figure out the correct powers for each wheel
        double flPower = (leftStickX + leftStickY + rightStickX);
        double frPower = (-leftStickX + leftStickY - rightStickX);
        double blPower = (-leftStickX + leftStickY + rightStickX);
        double brPower = (leftStickX + leftStickY - rightStickX);

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        if (maxPower<1) {
            maxPower = 1;
        }

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
    /*
     This gets the distance from any given target using the odometry
     It just uses the equation for the distance between two points
     The values need to be updated to use odometry once it's working
    */
    public double getDistanceFromTarget (Target target){
        double xDistance = 0;
        double zDistance = 0;
        return Math.hypot(xDistance - target.getX(), zDistance);
    }
    // This gets the correct launch angle for a target based off an average of the two nearest files which were stored during calibration.
    public double getLaunchAngle(Target target) {
        double distanceFromTarget = getDistanceFromTarget(target);
        /*
         Math.floor basically rounds down an integer. Both 6.1 and 6.8 would become 6.
         To make this work, 61 and 68 need to become 6.1 and 6.8, so we divide by 10.
        */
        double roundedDownDistance = Math.floor(distanceFromTarget / 10.0) * 10;
        //Same thing but the other way for Math.ceil
        double roundedUpDistance = Math.ceil(distanceFromTarget / 10.0) * 10;
        /*
         If the number is outside normal bounds, get the number closest to it and output that instead.
         Files are prefixed based on their target type (GOAL or POWERSHOT), so target.getTargetType() is used
         Target types are defined in the TargetType enum
         I'm not entirely sure if the .trims are necessary, but there's no harm in having them.
        */
        double launchAngleReal = 0;
        //If the number inputted is outside normal bounds, get the number closest to it.
        if (distanceFromTarget<60){
            launchAngleReal = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(target.getTargetType() + "60.txt")).trim());
            telemetry.addData("WARNING","distanceFromTarget is below 60. This shouldn't be happening in normal operation.");
            telemetry.speak("Warning");
            telemetry.update();
        } else if (distanceFromTarget>120) {
            launchAngleReal = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(target.getTargetType()+ "120.txt")).trim());
            telemetry.addData("WARNING","distanceFromTarget is above 120. This shouldn't be happening in normal operation.");
            telemetry.speak("Warning");
            telemetry.update();
            //Otherwise, act normally and average the two nearest files.
        } else {
            /*
             These retrieve the needed files stored during calibration.
             To continue the previous example, launchAngleLow would get the value stored in 60.txt while launchAngleHigh would get the value stored in 70.txt.
            */
            double launchAngleHigh = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(String.valueOf(target.getTargetType()) + roundedUpDistance + ".txt")).trim());
            double launchAngleLow = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(String.valueOf(target.getTargetType()) + roundedDownDistance + ".txt")).trim());
            // lowLaunchAngleWeight is the weight applied to the lower value as a multiplier. For 68, this would be .2, while for 61 is would be .9.
            double lowLaunchAngleWeight = .1 * (roundedUpDistance - distanceFromTarget);
            double highLaunchAngleWeight = 1 - lowLaunchAngleWeight;
            launchAngleReal = (lowLaunchAngleWeight * launchAngleLow) + (highLaunchAngleWeight * launchAngleHigh);
        }
        return launchAngleReal;
    }
}
