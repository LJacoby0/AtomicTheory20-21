package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ColorServoTest", group = "Remote")
public class ColorServoTest extends ServoTest{
    @Override
    public String getServoName() {
        return "sensor_servo";
    }
}
