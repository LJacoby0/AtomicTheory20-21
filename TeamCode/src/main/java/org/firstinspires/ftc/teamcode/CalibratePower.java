package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;


public abstract class CalibratePower extends OpMode {

    public static final double INCREMENT = .05;
    Robot rb = new Robot(telemetry);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public abstract TargetType getTargetType();

    private TargetType targetType = null;

    @Override
    public void init() {
        targetType = getTargetType();
    }

    boolean leftWasDown = false;
    boolean rightWasDown = false;
    boolean aWasDown = false;
    boolean triggerWasDown = false;
    double launchPower = 0.5;
    int minDistance = 60;
    int maxDistance = 120;


    int currentDistance = minDistance;

    @Override
    public void loop() {
        //Loop until all files 60-120 have been created
        while (currentDistance <= maxDistance) {
            telemetry.addData("Target Type", targetType);
            telemetry.addData("currentDistance", currentDistance);
            telemetry.addData("launchPower", launchPower);
            telemetry.update();

            //Shoot if the right trigger is pressed
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!triggerWasDown) {
                    triggerWasDown = true;
                    rb.shoot(launchPower, elapsedTime,true);
                } else {
                    rb.shoot(launchPower, elapsedTime,false);
                }
            } else {
                triggerWasDown = false;
            }
            /*If the a button is pressed, save the current angle to a file.
              All the "xWasDown" stuff is so none of this happens twice for just one button press.*/
            if (gamepad1.a) {
                if (!aWasDown) {
                    aWasDown = true;
                    ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(String.valueOf(targetType) + currentDistance + ".txt"), String.valueOf(launchPower));
                    currentDistance += 10;
                }
            } else {
                aWasDown = false;
            }
        }
        //Use the bumpers to change the angle. If the b button is pressed, change it slowly.
        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;
                if (gamepad1.b) {
                    launchPower -= .01;
                } else {
                    launchPower -= INCREMENT;
                }
            }
        } else {
            leftWasDown = false;
        }
        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
                if (gamepad1.b) {
                    launchPower += .01;
                } else {
                    launchPower += INCREMENT;
                }
            }
        } else {
            rightWasDown = false;
        }
        telemetry.clear();
        telemetry.addData("Done!","Press A to finish.");
        telemetry.update();

        if(gamepad1.a)
    {
        stop();
    }
}
}
