package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import static org.firstinspires.ftc.teamcode.Constants.*;
import org.opencv.core.Mat;

import java.util.Arrays;

@Autonomous(name = "Blue Super Auto", group = "Remote")
public class BlueSuperAuto extends LinearOpMode {
    Robot rb = new Robot(telemetry);
    static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int ringNumber;

    double distance1;
    double distance2;
    double distance3;
    @Override
    public void runOpMode(){
        rb.init(hardwareMap);
        //rb.wobbleGoalUp();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d(-62, 50, Math.toRadians(90));
        Trajectory trajectory1 = drive.trajectoryBuilder(startingPose)
                .strafeLeft(38)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(0.0, 5.5), Math.toRadians(0))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .strafeLeft(7.5)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeLeft(7.5)
                .build();
        Trajectory noRingStep1 = drive.trajectoryBuilder(trajectory4.end(), true)
                .splineTo(new Vector2d(12,48),Math.toRadians(90))
                .build();
        Trajectory noRingStep2 = drive.trajectoryBuilder(noRingStep1.end(), true)
                .strafeRight(18)
                .splineTo(new Vector2d(-36,44), Math.toRadians(270))
                .build();
        Trajectory noRingStep3 = drive.trajectoryBuilder(noRingStep2.end(), noRingStep2.end().getHeading() + Math.toRadians(180))
                .splineTo(new Vector2d(0,60), Math.toRadians(0))
                .build();
        Trajectory oneRingStep1 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(24, 36), 0.0)
                .build();
        Trajectory oneRingStep2 = drive.trajectoryBuilder(oneRingStep1.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .splineTo(new Vector2d(-10, 36), Math.toRadians(180))
                .splineTo(new Vector2d(-34, 36), Math.toRadians(180),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingStep3 = drive.trajectoryBuilder(oneRingStep2.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .strafeLeft(12)
                .build();
        Trajectory oneRingStep4 = drive.trajectoryBuilder(oneRingStep3.end())
                .splineTo(new Vector2d(0,36), Math.toRadians(0))
                .build();
        Trajectory oneRingStep5 = drive.trajectoryBuilder(oneRingStep4.end())
                .splineTo(new Vector2d(24,48), Math.toRadians(0))
                .build();
        Trajectory oneRingStep6 = drive.trajectoryBuilder(oneRingStep5.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .splineTo(new Vector2d(12,48), Math.toRadians(180))
                .build();
        Trajectory fourRingStep1 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(60, 48), Math.toRadians(270))
                .build();
        Trajectory fourRingStep2 = drive.trajectoryBuilder(fourRingStep1.end())
                .splineTo(new Vector2d(-10, 36), Math.toRadians(180))
                .splineTo(new Vector2d(-34, 36), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory fourRingStep3 = drive.trajectoryBuilder(fourRingStep2.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .strafeLeft(12)
                .build();
        Trajectory fourRingStep4 = drive.trajectoryBuilder(fourRingStep3.end())
                .splineTo(new Vector2d(0,36),Math.toRadians(0))
                .build();
        Trajectory fourRingStep5 = drive.trajectoryBuilder(fourRingStep4.end())
                .splineTo(new Vector2d(48,48), Math.toRadians(0))
                .build();
        Trajectory fourRingStep6 = drive.trajectoryBuilder(fourRingStep5.end().plus(new Pose2d(0,0, Math.toRadians(270))))
                .splineTo(new Vector2d(12,12),Math.toRadians(180))
                .build();
        waitForStart();
        drive.followTrajectory(trajectory1);
        double distance = rb.colorSensor.getDistance(DistanceUnit.CM);
        if (distance < 26) {
            ringNumber = 4;
            distance1 = distance;
        } else {
            rb.sensor_servo.setPosition(Constants.COLOR_SERVO_DOWN);
            //Be very careful, this might cause problems and need to be implemented differently
            timer.reset();
            if (timer.time()>1000){
                if (distance <= 26) {
                    ringNumber = 1;
                    distance2 = distance;
                } else {
                    ringNumber = 0;
                    distance3 = distance;
                }
            }
        }
        drive.followTrajectory(trajectory2);
        //The time it takes for the robot to shoot one ring
        int ONE_SHOT_MILLIS = 750;
        rb.getPowerAndShoot(trajectory2.end(), bluePowershot3, timer,true,false);
        while (timer.time() < ONE_SHOT_MILLIS){
            rb.getPowerAndShoot(trajectory2.end(), bluePowershot3, timer,false,false);
        }
        rb.hammerIn();
        drive.followTrajectory(trajectory3);
        rb.getPowerAndShoot(trajectory3.end(), bluePowershot2, timer,true,false);
        while (timer.time() < ONE_SHOT_MILLIS){
            rb.getPowerAndShoot(trajectory3.end(), bluePowershot2, timer,false,false);
        }
        rb.hammerIn();
        drive.followTrajectory(trajectory4);
        timer.reset();
        rb.getPowerAndShoot(trajectory4.end(), bluePowershot1, timer,true,false);
        while (timer.time() < ONE_SHOT_MILLIS){
            rb.getPowerAndShoot(trajectory4.end(), bluePowershot1, timer,false,false);
        }
        rb.stopFlywheel();
        rb.hammerIn();
        switch (ringNumber){
            case 0:
                drive.followTrajectory(noRingStep1);
                //rb.wobbleGoalDown
                drive.followTrajectory(noRingStep2);
                //rb.wobbleGoalUp
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(noRingStep3);
                //rb.wobbleGoalDown
                break;
            case 1:
                drive.followTrajectory(oneRingStep1);
                drive.turn(Math.toRadians(180));
                //rb.wobbleGoalDown
                rb.intakeMotor.setPower(1);
                drive.followTrajectory(oneRingStep2);
                rb.intakeMotor.setPower(0);
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(oneRingStep3);
                //rb.wobbleGoalUp
                drive.followTrajectory(oneRingStep4);
                rb.getPowerAndShoot(fourRingStep4.end(), blueTopGoal, timer,true,false);
                while (timer.time() < ONE_SHOT_MILLIS){
                    rb.getPowerAndShoot(fourRingStep4.end(), blueTopGoal, timer,false,false);
                }
                rb.hammerIn();
                drive.followTrajectory(oneRingStep5);
                drive.turn(180);
                drive.followTrajectory(oneRingStep6);
            case 4:
                drive.followTrajectory(fourRingStep1);
                //rb.wobbleGoalDown
                rb.intakeMotor.setPower(1);
                drive.followTrajectory(fourRingStep2);
                rb.intakeMotor.setPower(0);
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(fourRingStep3);
                //rb.wobbleGoalUp
                drive.followTrajectory(fourRingStep4);
                rb.getPowerAndShoot(fourRingStep4.end(), blueTopGoal, timer,true,false);
                while (timer.time() < 2000){
                    rb.getPowerAndShoot(fourRingStep4.end(), blueTopGoal, timer,false,false);
                }
                rb.hammerIn();
                drive.followTrajectory(fourRingStep5);
                //rb.wobbleGoalDown
                drive.followTrajectory(fourRingStep6);
        }
    }
}
