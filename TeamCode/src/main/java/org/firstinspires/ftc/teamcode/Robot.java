package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class Robot {
    //Declare motors
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    LinearOpMode opMode;

    void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

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
            frontLeftMotor.setPower((leftStickX + leftStickY + rightStickX) * sensitivity);
            frontRightMotor.setPower((-leftStickX + leftStickY - rightStickX) * sensitivity);
            backLeftMotor.setPower((-leftStickX + leftStickY + + rightStickX) * sensitivity);
            backRightMotor.setPower((leftStickX + leftStickY - rightStickX) * sensitivity);
    }
    void driveStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
