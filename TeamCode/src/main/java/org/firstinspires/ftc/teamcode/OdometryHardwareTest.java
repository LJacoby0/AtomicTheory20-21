package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Odometry System Hardware Check", group = "Calibration")
public class OdometryHardwareTest extends OpMode {
    private Robot rb = new Robot(telemetry);
    @Override
    public void init() {
        rb.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Encoder Value", rb.leftEncoder.getCurrentPosition());
        telemetry.addData("Horizontal Encoder Value", rb.middleEncoder.getCurrentPosition());
        telemetry.update();

    }
}
