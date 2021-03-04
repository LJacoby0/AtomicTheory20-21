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
    double distance1;
    double distance2;
    double distance3;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap);
        waitForStart();
        timer.reset();
        double distance = rb.colorSensor.getDistance(DistanceUnit.CM);
        if (distance < 26) {
            ringNumber = 4;
            distance1 = distance;
        } else {
            rb.sensor_servo.setPosition(Constants.COLOR_SERVO_DOWN);
            //Be very careful, this might cause problems and need to be implemented differently
            timer.reset();
            if (timer.time()>1000){
                if (distance <= 26) {
                    ringNumber = 1;
                    distance2 = distance;
                } else {
                    ringNumber = 0;
                    distance3 = distance;
                }
            }
        }
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Number of Rings", ringNumber);
            telemetry.addData("Distance 1", distance1);
            telemetry.addData("Distance 2", distance2);
            telemetry.addData("Distance 3", distance3);
            telemetry.addData("Current Distance", distance);
            telemetry.update();
        }
    }
}
