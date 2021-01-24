package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Park", group = "Remote")
public class RoadRunnerPark extends LinearOpMode {
    Robot rb = new Robot(telemetry);
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d(-62, 50, Math.toRadians(90));
        Trajectory newTrajectory = drive.trajectoryBuilder(startingPose)
                .splineTo(new Vector2d(6, 50), Math.toRadians(90))
                .build();
        waitForStart();
        drive.followTrajectory(newTrajectory);
    }
}
