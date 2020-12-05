package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CalibratePowershot", group = "Calibrate")
public class CalibratePowershot extends CalibratePower {
    @Override
    public TargetType getTargetType() {
        return TargetType.POWERSHOT;
    }
}
