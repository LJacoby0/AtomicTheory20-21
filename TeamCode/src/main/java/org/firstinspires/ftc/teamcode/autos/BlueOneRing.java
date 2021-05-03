package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.HAMMER_SERVO_ROTATION_TIME;

@Autonomous(name = "Blue One Ring", group = "Remote")
public class BlueOneRing extends LinearOpMode {
    Robot rb = new Robot(telemetry);
    static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int ringNumber;
    int pickupWheelVel = 20;
    double hue1;
    double hue2;
    double angleDifference;
    public static final double ANGLE_TOLERANCE = .05;
    final float[] hsvValues = new float[3];
    int goalPower = 1800;

    @Override
    public void runOpMode() throws InterruptedException{
        rb.init(hardwareMap);
        rb.wobbleGoalUp();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d(-62, 25, Math.toRadians(270));
        Trajectory trajectory1 = drive.trajectoryBuilder(startingPose)
                .strafeLeft(38)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(6.0)
                .splineTo(new Vector2d(4.0,36.0),Math.toRadians(90.0))
                .build();
        Trajectory noRingStep1 = drive.trajectoryBuilder(trajectory4.end(), true)
                .splineTo(new Vector2d(0.0, 52.0), Math.toRadians(90.0))
                .build();
        Trajectory noRingStep2 = drive.trajectoryBuilder(noRingStep1.end())
                .strafeRight(6.0)
                .build();
        Trajectory noRingStep3 = drive.trajectoryBuilder(noRingStep2.end())
                .splineTo(new Vector2d(-30, 54), Math.toRadians(90))
                .build();
        Trajectory noRingStep4 = drive.trajectoryBuilder(noRingStep3.end())
                .strafeLeft(7,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory noRingStep5 = drive.trajectoryBuilder(noRingStep4.end())
                .splineTo(new Vector2d(-8.0, 52.0), Math.toRadians(270.0))
                .build();
        Trajectory noRingStep6 = drive.trajectoryBuilder(noRingStep5.end())
                .strafeRight(4.0)
                .build();
        Trajectory noRingStep7 = drive.trajectoryBuilder(noRingStep6.end())
                .splineTo(new Vector2d(12.0, 30.0), Math.toRadians(0.0))
                .build();
        Trajectory oneRingStep1 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(36, 24), 0.0)
                .build();
        Trajectory oneRingStep2 = drive.trajectoryBuilder(oneRingStep1.end())
                .strafeRight(4)
                .build();
        Trajectory oneRingStep3 = drive.trajectoryBuilder(oneRingStep1.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .splineTo(new Vector2d(-10, 36), Math.toRadians(180))
                .splineTo(new Vector2d(-30, 36), Math.toRadians(180),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(pickupWheelVel, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(new Vector2d(-30, 52), Math.toRadians(90))
                .build();
        Trajectory oneRingStep4 = drive.trajectoryBuilder(oneRingStep3.end())
                .strafeLeft(7,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingStep5 = drive.trajectoryBuilder(oneRingStep4.end())
                .splineTo(new Vector2d(0,36), Math.toRadians(0))
                .build();
        Trajectory oneRingStep6 = drive.trajectoryBuilder(oneRingStep5.end())
                .splineTo(new Vector2d(24,48), Math.toRadians(0))
                .build();
        Trajectory oneRingStep7 = drive.trajectoryBuilder(oneRingStep6.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .strafeRight(6)
                .build();
        Trajectory oneRingStep8 = drive.trajectoryBuilder(oneRingStep7.end())
                .splineTo(new Vector2d(12,54), Math.toRadians(180))
                .build();
        Trajectory fourRingStep1 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(60, 44), Math.toRadians(0))
                .build();
        Trajectory fourRingStep2 = drive.trajectoryBuilder(fourRingStep1.end())
                .strafeRight(4)
                .build();
        Trajectory fourRingStep3 = drive.trajectoryBuilder(fourRingStep2.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .splineTo(new Vector2d(-10, 36), Math.toRadians(180))
                .splineTo(new Vector2d(-34, 36), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(pickupWheelVel, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(new Vector2d(-30, 52), Math.toRadians(90.0))
                .build();
        Trajectory fourRingStep4 = drive.trajectoryBuilder(fourRingStep3.end())
                .strafeLeft(7,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory fourRingStep5 = drive.trajectoryBuilder(fourRingStep4.end())
                .splineTo(new Vector2d(0,36),Math.toRadians(0))
                .build();
        Trajectory fourRingStep6 = drive.trajectoryBuilder(fourRingStep5.end())
                .splineTo(new Vector2d(47,50), Math.toRadians(270))
                .build();
        Trajectory fourRingStep7 = drive.trajectoryBuilder(fourRingStep6.end())
                .strafeRight(4)
                .build();
        Trajectory fourRingStep8 = drive.trajectoryBuilder(fourRingStep7.end())
                .splineTo(new Vector2d(12,12),Math.toRadians(180))
                .build();
        drive.setPoseEstimate(startingPose);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectory1);
            int color = rb.colorSensor.getNormalizedColors().toColor();
            Color.colorToHSV(color, hsvValues);
            if (hsvValues[0] > 24 && hsvValues[0] < 100) {
                ringNumber = 4;
                hue1 = hsvValues[0];
            } else {
                rb.sensor_servo.setPosition(Constants.COLOR_SERVO_DOWN);
                Thread.sleep(1000);
                color = rb.colorSensor.getNormalizedColors().toColor();
                Color.colorToHSV(color, hsvValues);
                if (hsvValues[0] > 24 && hsvValues[0]< 100) {
                    ringNumber = 1;
                    hue2 = hsvValues[0];
                } else {
                    ringNumber = 0;
                }
            }
            //Use this to run a certain auto

        ringNumber = 1;
            drive.followTrajectory(trajectory4);
//            angleDifference = drive.getPoseEstimate().getHeading() - rb.getAngleToTarget(drive.getPoseEstimate(), bluePowershot3);
//            if (Math.abs(angleDifference) > ANGLE_TOLERANCE) {
//                drive.turn(angleDifference);
//            }
            shoot(/*trajectory2.end(), bluePowershot3*/goalPower, timer);
            //The time it takes for the robot to shoot one ring
//        int ONE_SHOT_MILLIS = 750;
//        rb.getPowerAndShoot(trajectory2.end(), bluePowershot3, timer,true);
//        while (timer.time() < ONE_SHOT_MILLIS){
//            rb.getPowerAndShoot(trajectory2.end(), bluePowershot3, timer,false);
//        }
//        rb.hammerIn();
            shoot(/*trajectory3.end(), bluePowershot2*/goalPower, timer);
//        rb.getPowerAndShoot(trajectory3.end(), bluePowershot2, timer,true);
//        while (timer.time() < ONE_SHOT_MILLIS){
//            rb.getPowerAndShoot(trajectory3.end(), bluePowershot2, timer,false);
//        }
//        rb.hammerIn();
            shoot(/*trajectory4.end(), bluePowershot1*/goalPower, timer);
//        timer.reset();
//        rb.getPowerAndShoot(trajectory4.end(), bluePowershot1, timer,true);
//        while (timer.time() < ONE_SHOT_MILLIS){
//            rb.getPowerAndShoot(trajectory4.end(), bluePowershot1, timer,false);
//        }
            shoot(/*trajectory2.end(), bluePowershot3*/goalPower, timer);

            rb.stopFlywheel();
            rb.hopperDown();
            switch (ringNumber){
                case 0:
                    drive.followTrajectory(noRingStep1);
                    rb.wobbleGoalDown();
                    drive.followTrajectory(noRingStep2);
                    drive.followTrajectory(noRingStep3);
                    drive.followTrajectory(noRingStep4);
                    rb.wobbleGoalUp();
                    drive.followTrajectory(noRingStep5);
                    rb.wobbleGoalDown();
                    drive.followTrajectory(noRingStep6);
                    rb.wobbleGoalUp();
                    drive.followTrajectory(noRingStep7);
                    break;
                case 1:
                    drive.followTrajectory(oneRingStep1);
                    rb.wobbleGoalDown();
                    drive.followTrajectory(oneRingStep2);
                    drive.turn(Math.toRadians(180));
                    rb.intakeMotor.setPower(-1);
                    drive.followTrajectory(oneRingStep3);
                    rb.intakeMotor.setPower(0);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(oneRingStep4);
                    rb.wobbleGoalUp();
                    drive.followTrajectory(oneRingStep5);
                    rb.hopperUp();
                    shoot(/*oneRingStep5.end(), blueTopGoal*/goalPower, timer);
                    rb.stopFlywheel();
                    drive.followTrajectory(oneRingStep6);
                    drive.turn(Math.toRadians(180));
                    rb.wobbleGoalDown();
                    drive.followTrajectory(oneRingStep7);
                    rb.wobbleGoalUp();
                    drive.followTrajectory(oneRingStep8);
                    break;
                case 4:
                    drive.followTrajectory(fourRingStep1);
                    rb.wobbleGoalDown();
                    drive.followTrajectory(fourRingStep2);
                    rb.intakeMotor.setPower(-1);
                    drive.followTrajectory(fourRingStep3);
                    rb.intakeMotor.setPower(0);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(fourRingStep4);
                    rb.wobbleGoalUp();
                    drive.followTrajectory(fourRingStep5);
                    rb.hopperUp();
                    shoot(/*fourRingStep5.end(), blueTopGoal*/goalPower, timer);
                    shoot(/*oneRingStep5.end(), blueTopGoal*/goalPower, timer);
                    shoot(/*oneRingStep5.end(), blueTopGoal*/goalPower, timer);
                    shoot(/*oneRingStep5.end(), blueTopGoal*/goalPower, timer);
                    rb.stopFlywheel();
                    drive.followTrajectory(fourRingStep6);
                    rb.wobbleGoalDown();
                    drive.followTrajectory(fourRingStep8);
                    rb.wobbleGoalUp();
                    break;
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
            telemetry.addData("Auto","Finished!");
            telemetry.update();
            Thread.sleep(1000);
            stop();
        }
    }
    //This shoots one ring. Ignore the android studio warning, it just doesn't like what I've done.
    //Basically, rb.shootRing continuously outputs false until a ring shoots, then it outputs true
    //However, after that, we still have to wait to bring the hammer back, otherwise it might not fully push the ring.
    //If we're low on time, I can technically have it bring the hammer back during movement, but that's not easy
    private void shoot(double power, @NotNull ElapsedTime timer) throws InterruptedException {
        timer.reset();
        while (!rb.shootRing(timer, power));
        Thread.sleep(HAMMER_SERVO_ROTATION_TIME);
        rb.hammerBack();
    }
}
