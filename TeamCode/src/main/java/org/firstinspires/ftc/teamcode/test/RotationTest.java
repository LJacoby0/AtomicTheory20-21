package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hopper Test", group = "Remote")
public class RotationTest extends ServoTest{
    @Override
    public String getServoName() {
        return "hopper_rotate";
    }
}
