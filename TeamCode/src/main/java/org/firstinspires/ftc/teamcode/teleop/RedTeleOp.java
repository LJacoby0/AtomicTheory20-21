package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.UGAngleHighGoalPipeline;

@TeleOp(name = "Red New TeleOp", group = "Remote")
public class RedTeleOp extends RemoteTeleOp{

    @Override
    public UGAngleHighGoalPipeline.Target getColor() {
        return UGAngleHighGoalPipeline.Target.RED;
    }
}
