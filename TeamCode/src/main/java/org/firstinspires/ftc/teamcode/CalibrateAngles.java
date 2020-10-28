package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


class CalibrateAngles extends OpMode {

    public static final double INCREMENT = .05;

    @Override
    public void init() {
    }
    boolean leftWasDown = false;
    boolean rightWasDown = false;
    boolean aWasDown = false;
    public double launchAngle = 0.5;
    public int currentDistance = 60;

    @Override
    public void loop(){
        //Loop until all files 60-120 have been created
        while (currentDistance <= 120){
            telemetry.addData("currentDistance",currentDistance);
            telemetry.addData("launchAngle", launchAngle);
            telemetry.update();

            //This will eventually implement the robot and allow for shooting during calibration
            if (gamepad1.right_trigger >= Constants.TRIGGER_THRESHOLD) {
                //rb.shoot(launchAngle);
            }
            /*If the a button is pressed, save the current angle to a file.
              All the "xWasDown" stuff is so none of this happens twice for just one button press.*/
            if (gamepad1.a) {
                if (!aWasDown) {
                    aWasDown = true;
                    ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(currentDistance + ".txt"), String.valueOf(launchAngle));
                    currentDistance += 10;
                } else {
                    aWasDown = false;
                }
            }
            //Use the bumpers to change the angle. If the b button is pressed, change it slowly.
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
        telemetry.clear();
        telemetry.addData("Done!", "Press A to finish.");
        telemetry.update();
        if (gamepad1.a){
            stop();
        }
    }
}
