package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;

class Robot {
    //Declare motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Telemetry telemetry;
    InterpLUT goalLut = new InterpLUT();
    InterpLUT powershotLut = new InterpLUT();

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

        createCalibrationInterplut(goalLut, TargetType.GOAL);
        createCalibrationInterplut(powershotLut, TargetType.POWERSHOT);
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
    public Double getLaunchAngle(Target target) {
        //If we're shooting at a goal, get it from the goal interplut, otherwise get it from the powershot. The other one is just a sanity check.
        if (target.getTargetType() == TargetType.GOAL){
            return goalLut.get(getDistanceFromTarget(target));
        } else if(target.getTargetType() == TargetType.POWERSHOT){
            return powershotLut.get(getDistanceFromTarget(target));
        }
        else {
            telemetry.speak("Something's Wrong!");
            telemetry.update();
            return null;
        }
    }
    private void createCalibrationInterplut(InterpLUT lut, TargetType targetType){
        //Read the files we made in CalibrateGoal/Powershot, and make an interplut out of them.
        for (int i = 60; i <= 120; i+=10) {
            try {
                lut.add(i, Double.parseDouble(ReadWriteFile.readFileOrThrow(AppUtil.getInstance().getSettingsFile(String.valueOf(targetType)+i+".txt"))));
            } catch (IOException e) {
                telemetry.speak("Warning: Calibration Failed");
                telemetry.addData("Calibration","Failed");
                telemetry.update();
            }
        }
    }
}
