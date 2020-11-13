package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Remote TeleOp", group = "Remote")
public class RemoteTeleOp extends OpMode {
    private Robot rb = new Robot();
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
        telemetry.addData("Status", "Looping");
        telemetry.update();
        driveChassis();
    }

    private void driveChassis() {
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
