package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TargetType;

@TeleOp(name = "CalibrateGoal", group = "Calibrate")
public class CalibrateGoal extends CalibratePower {
    @Override
    public TargetType getTargetType() {
        return TargetType.GOAL;
    }
}
