package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


public abstract class ServoTest extends OpMode {

    public static final double INCREMENT = .05;
//    Robot rb = new Robot(telemetry);
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    boolean leftWasDown = false;
    boolean rightWasDown = false;
    boolean aWasDown = false;
    double servoPosition = .5;

    @Override
    public void loop() {
        telemetry.addData("launchAngle", servoPosition);
        telemetry.update();

        if (gamepad1.a) {
            if (!aWasDown) {
                aWasDown = true;
                servo.setPosition(servoPosition);
            }
        } else {
                aWasDown = false;
            }
        //Use the bumpers to change the angle. If the b button is pressed, change it slowly.
        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;
                if (gamepad1.b) {
                    servoPosition -= .01;
                } else {
                    servoPosition -= INCREMENT;
                }
            }
        } else {
            leftWasDown = false;
        }
        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
                if (gamepad1.b) {
                    servoPosition += .01;
                } else {
                    servoPosition += INCREMENT;
                }
            }
        } else {
            rightWasDown = false;
        }
        telemetry.clear();
        telemetry.addData("Done!", "Press A to finish.");
        telemetry.update();
        if (gamepad1.a) {
            stop();
        }
    }
}
