package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Advanced TeleOp", group = "Remote")
public class RedAdvancedTeleOp extends AdvancedTeleOp {
    @Override
    public Target[] getTargets() {
        return new Target[]{Constants.redTopGoal, Constants.redPowershot1, Constants.redPowershot2, Constants.redPowershot3};
    }
}
