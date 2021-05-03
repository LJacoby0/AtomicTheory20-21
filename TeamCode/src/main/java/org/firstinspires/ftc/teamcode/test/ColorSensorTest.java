package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Color Detection Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    private Robot rb = new Robot(telemetry);
    int ringNumber;
    double hue1;
    double hue2;
    final float[] hsvValues = new float[3];
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap);
        waitForStart();
        timer.reset();
        int color = rb.colorSensor.getNormalizedColors().toColor();
        Color.colorToHSV(color, hsvValues);
        if (hsvValues[0] > 24) {
            ringNumber = 4;

            hue1 = hsvValues[0];
        } else {
            rb.sensor_servo.setPosition(Constants.COLOR_SERVO_DOWN);
            Thread.sleep(1000);
            color = rb.colorSensor.getNormalizedColors().toColor();
            Color.colorToHSV(color, hsvValues);
                if (hsvValues[0] > 24) {
                    ringNumber = 1;
                    hue2 = hsvValues[0];
                } else {
                    ringNumber = 0;
                }
            }
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Number of Rings", ringNumber);
            telemetry.addData("Distance 1", hue1);
            telemetry.addData("Distance 2", hue2);
            telemetry.addData("Current Distance", rb.colorSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
