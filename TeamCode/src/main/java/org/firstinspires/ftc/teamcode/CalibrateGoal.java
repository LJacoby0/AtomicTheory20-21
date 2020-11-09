package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CalibrateGoal", group = "Calibrate")
class CalibrateGoal extends CalibrateAngles{
    @Override
    public TargetType getTargetType() {
        return TargetType.GOAL;
    }
}
