package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Advanced TeleOp", group = "Remote")
public class BlueAdvancedTeleOp extends AdvancedTeleOp {
    @Override
    public Target[] getTargets() {
        return new Target[]{Constants.blueTopGoal, Constants.bluePowershot1, Constants.bluePowershot2, Constants.bluePowershot3};
    }
}
