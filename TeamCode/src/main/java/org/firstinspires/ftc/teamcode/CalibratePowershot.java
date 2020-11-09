package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CalibratePowershot", group = "Calibrate")
class CalibratePowershot extends CalibrateAngles{
    @Override
    public TargetType getTargetType() {
        return TargetType.POWERSHOT;
    }
}
