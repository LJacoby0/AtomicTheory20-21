package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Remote TeleOp", group = "Remote")
public class RemoteTeleOp extends OpMode {
    private Robot rb = new Robot(telemetry);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        rb.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveChassis();
        shootTarget();
        telemetry.addData("Status", "Looping");
        telemetry.update();
    }

    int targetInt = 0;
    boolean rbWasDown;
    boolean lbWasDown;
    boolean triggerWasDown;
    Target[] targets = new Target[]{Constants.blueTopGoal, Constants.bluePowershot1, Constants.bluePowershot2, Constants.bluePowershot3};

    private void shootTarget() {
        //This code cycles through the target list using the bumpers
        if (gamepad1.right_bumper) {
            if (!rbWasDown) {
                rbWasDown = true;
                targetInt += 1;
            }
        } else {
            rbWasDown = false;
        }
        if (gamepad1.left_bumper) {
            if (!lbWasDown) {
                lbWasDown = true;
                targetInt -= 1;
            }
        } else {
            lbWasDown = false;
        }
        if (targetInt<0){
            targetInt = targets.length-1;
        } else if (targetInt>targets.length-1){
            targetInt = 0;
        }
        telemetry.addData("Current Target:", targets[targetInt]);
        //This tells the command whether or not it's the first time the button has been pressed.
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (!triggerWasDown) {
                triggerWasDown = true;
                rb.shoot(Constants.POWER_CONSTANT, elapsedTime, true);
            } else {
                rb.shoot(Constants.POWER_CONSTANT, elapsedTime, false);
            }
        } else {
            triggerWasDown = false;
        }
    }

        private void driveChassis () {
            float leftStickY = -gamepad1.left_stick_y;
            float leftStickX = gamepad1.left_stick_x;
            float rightStickX = gamepad1.right_stick_x;

            double pow;
            if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
                pow = DRIVE_POWER_SLOW;
            } else {
                pow = DRIVE_POWER;
            }
            if (Math.abs(leftStickX) + Math.abs(leftStickY) >= DRIVE_STICK_THRESHOLD || Math.abs(rightStickX) >= DRIVE_STICK_THRESHOLD) {
                rb.drive(leftStickX, leftStickY, rightStickX, pow);
            } else {
                rb.driveStop();
            }
        }
    }
