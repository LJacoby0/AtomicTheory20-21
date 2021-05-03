package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TargetType;

@TeleOp(name = "CalibratePowershot", group = "Calibrate")
public class CalibratePowershot extends CalibratePower {
    @Override
    public TargetType getTargetType() {
        return TargetType.POWERSHOT;
    }
}
