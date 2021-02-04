package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Detection Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    private Robot rb = new Robot(telemetry);
    int ringNumber;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap);
        waitForStart();
        timer.reset();
        double distance = rb.colorSensor.getDistance(DistanceUnit.CM);
        if (distance < 26) {
            ringNumber = 4;
        } else {
            rb.sensor_servo.setPosition(Constants.COLOR_SERVO_DOWN);
            //Be very careful, this might cause problems and need to be implemented differently
            wait(1000);
            if (distance <= 26) {
                ringNumber = 1;
            } else {
                ringNumber = 0;
            }
        }
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Number of Rings", ringNumber);
            telemetry.update();
        }
    }
}
