package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RotationTest", group = "Remote")
public class RotationTest extends ServoTest{
    @Override
    public String getServoName() {
        return "hopper_rotate";
    }
}
