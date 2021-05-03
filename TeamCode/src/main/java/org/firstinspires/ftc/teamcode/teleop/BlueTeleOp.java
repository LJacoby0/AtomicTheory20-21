package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.UGAngleHighGoalPipeline;

@TeleOp(name = "Blue New TeleOp", group = "Remote")
public class BlueTeleOp extends RemoteTeleOp {
    @Override
    public UGAngleHighGoalPipeline.Target getColor() {
        return UGAngleHighGoalPipeline.Target.BLUE;
    }
}
