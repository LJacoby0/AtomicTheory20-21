package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CalibrateGoal", group = "Calibrate")
public class CalibrateGoal extends CalibratePower {
    @Override
    public TargetType getTargetType() {
        return TargetType.GOAL;
    }
}