package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Wobble Goal Test", group = "Remote")
public class WobbleTest extends OpMode {
    Robot rb = new Robot(telemetry);
    public static final int INCREMENT = 100;
    public static final int SMALL_INCREMENT = 5;
    @Override
    public void init() {
        rb.init(hardwareMap);
    }

    boolean leftWasDown = false;
    boolean rightWasDown = false;
    boolean aWasDown = false;
    int encoderTargetPosition = 0;

    @Override
    public void loop() {
        telemetry.addData("Set position:", encoderTargetPosition);
        telemetry.addData("Position on motor:", rb.wobbleMotor.getTargetPosition());
        telemetry.addData("Observed position", rb.wobbleMotor.getCurrentPosition());
        telemetry.update();

        if (gamepad1.a) {
            rb.wobbleMotor.setTargetPosition(encoderTargetPosition);
            rb.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.wobbleMotor.setPower(1);
            aWasDown = true;
        }
        //Use the bumpers to change the angle. If the b button is pressed, change it slowly.
        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;
                if (gamepad1.b) {
                    encoderTargetPosition -= SMALL_INCREMENT;
                } else {
                    encoderTargetPosition -= INCREMENT;
                }
            }
        } else {
            leftWasDown = false;
        }
        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
                if (gamepad1.b) {
                    encoderTargetPosition += SMALL_INCREMENT;
                } else {
                    encoderTargetPosition += INCREMENT;
                }
            }
        } else {
            rightWasDown = false;
        }
    }
}
