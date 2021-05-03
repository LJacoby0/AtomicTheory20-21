package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.TargetType;

import static org.firstinspires.ftc.teamcode.Constants.CALIBRATION_INTERVAL;
import static org.firstinspires.ftc.teamcode.Constants.FLYWHEEL_MAX_VELO;
import static org.firstinspires.ftc.teamcode.Constants.MAXIMUM_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.MINIMUM_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;


public abstract class CalibratePower extends LinearOpMode {

    public static final double INCREMENT = 50;
    public static final double SMALL_INCREMENT = 5;
    Robot rb = new Robot(telemetry);
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public abstract TargetType getTargetType();

    private TargetType targetType = null;

    boolean leftWasDown = false;
    boolean rightWasDown = false;
    boolean xWasDown = false;
    boolean yWasDown = false;
    boolean aWasDown = false;
    boolean triggerWasDown = false;
    int launchVelocity = FLYWHEEL_MAX_VELO;


    int currentDistance = MINIMUM_DISTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        targetType = getTargetType();
        rb.init(hardwareMap);
        waitForStart();
        while (currentDistance <= MAXIMUM_DISTANCE && opModeIsActive()) {
            telemetry.addData("Target Type", targetType);
            telemetry.addData("currentDistance", currentDistance);
            telemetry.addData("Target Velocity", launchVelocity);
            telemetry.addData("Measured Velocity", rb.flywheelMotor.getVelocity());
            telemetry.update();

            //Shoot if the right trigger is pressed
            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!triggerWasDown) {
                    triggerWasDown = true;
                    rb.shoot(launchVelocity, elapsedTime,true);
                } else {
                    rb.shoot(launchVelocity, elapsedTime,false);
                }
            } else {
                triggerWasDown = false;
                rb.stopFlywheel();
                rb.hammerBack();
            }
            /*If the a button is pressed, save the current angle to a file.
              All the "xWasDown" stuff is so none of this happens twice for just one button press.*/
            if (gamepad1.a) {
                if (!aWasDown) {
                    aWasDown = true;
                    ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(String.valueOf(targetType) + currentDistance + ".txt"), String.valueOf(launchVelocity));
                    currentDistance += CALIBRATION_INTERVAL;
                }
            } else {
                aWasDown = false;
            }
            if (gamepad1.left_bumper) {
                if (!leftWasDown) {
                    leftWasDown = true;
                    if (gamepad1.b) {
                        this.launchVelocity -= SMALL_INCREMENT;
                    } else {
                        this.launchVelocity -= INCREMENT;
                    }
                }
            } else {
                leftWasDown = false;
            }
            if (gamepad1.right_bumper) {
                if (!rightWasDown) {
                    rightWasDown = true;
                    if (gamepad1.b) {
                        this.launchVelocity += SMALL_INCREMENT;
                    } else {
                        this.launchVelocity += INCREMENT;
                    }
                }
            } else {
                rightWasDown = false;
            }

            if (gamepad1.x) {
                rb.hammerBack();
            }
            if (gamepad1.y){
                rb.hammerPush();
            }
        }
        telemetry.clear();
        telemetry.addData("Done!","Press A to finish.");
        telemetry.update();

        if(gamepad1.a) {
            stop();
        }



        //Loop until all files 50-120 have been created
        //Use the bumpers to change the angle. If the b button is pressed, change it slowly.

    }
}
