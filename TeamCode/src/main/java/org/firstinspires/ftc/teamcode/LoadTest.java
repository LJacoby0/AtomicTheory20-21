package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LoadServoTest", group = "Remote")
public class LoadTest extends ServoTest{
    @Override
    public String getServoName() {
        return "load";
    }
}
