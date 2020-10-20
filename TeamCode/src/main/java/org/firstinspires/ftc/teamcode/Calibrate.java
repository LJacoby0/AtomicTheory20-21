package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


class Calibrate extends OpMode {

    public static final double INCREMENT = .05;

    @Override
    public void init() {
        // Initiate odometry
    }
    @Override
    public void start() {
    }

    public boolean leftWasDown = false;
    public boolean rightWasDown = false;
    public double launchAngle = 0.5;
    public double currentDistance = 60;
    File store = new File("calibration.csv");

    @Override
    public void loop(){
        telemetry.addData("currentDistance",currentDistance);
        telemetry.addData("launchAngle", launchAngle);

        if (gamepad1.right_trigger >= Constants.TRIGGER_THRESHOLD) {
            //rb.shoot(launchAngle);
        }
        if (gamepad1.a){
            try {
                FileWriter fileWriter = new FileWriter(store);
                fileWriter.append(launchAngle + "," + currentDistance + "\n");
                currentDistance+=10;
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;
                if (gamepad1.b) {
                    launchAngle -= .01;
                } else {
                    launchAngle -= INCREMENT;
                }
            }
        } else {
            leftWasDown = false;
        }
        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
                if (gamepad1.b) {
                    launchAngle += .01;
                } else {
                    launchAngle += INCREMENT;
                }

            }
        } else {
            rightWasDown = false;
        }
    }
}
