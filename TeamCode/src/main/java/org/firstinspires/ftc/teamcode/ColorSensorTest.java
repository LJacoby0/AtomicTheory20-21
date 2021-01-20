package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Color Detection Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    private Robot rb = new Robot(telemetry);
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap);
        waitForStart();
        double ringNumber = rb.getRingNumber(timer);
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Number of Rings", ringNumber);
            telemetry.update();
        }
    }
}
