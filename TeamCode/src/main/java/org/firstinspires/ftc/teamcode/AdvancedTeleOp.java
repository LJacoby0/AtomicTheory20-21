package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.HAMMER_VERY_IN;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.WOBBLE_GOAL_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.WOBBLE_GOAL_UP;

@TeleOp(name = "Advanced TeleOp", group = "Remote")
public class AdvancedTeleOp extends OpMode {
    //This will return either a list of blue targets or a list of red targets
    //Angle tolerance, in radians
    public static final double ANGLE_TOLERANCE = .05;
    private Robot rb = new Robot(telemetry);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Pose2d shootingLocation;
    boolean positionSet = false;
    int targetInt = 0;
    Target currentTarget = Constants.blueTopGoal;

    //Debouncing variables
    boolean dpadLWasDown;
    boolean dpadRWasDown;
    boolean triggerWasDown;
    boolean bWasDown;

    boolean isAutoAim = true;


    public enum DriveState{
        MANUAL,
        AUTOMATIC,
        ADJUST_WOBBLE
    }
    DriveState driveState = DriveState.MANUAL;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        //Drive has to be initialized like this
        drive = new SampleMecanumDrive(hardwareMap);
        //This needs to be set to our actual position once we get it.
        if (!PoseStorage.currentPose.equals(new Pose2d())) {
            drive.setPoseEstimate(PoseStorage.currentPose);
        } else {
            //This is just a fallback for if we're testing this in a not match scenario,
            //it's the corner of the field facing towards the goals
            drive.setPoseEstimate(new Pose2d(63,63,0));
        }
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        rb.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void loop() {
//        telemetry.addData("Targets",targets);
        Pose2d poseEstimate = drive.getPoseEstimate();
//        if (gamepad1.dpad_up){
//            isAutoAim = true;
//        } else if (gamepad1.dpad_down) {
//            isAutoAim = false;
//        }
        switch (driveState){
            case MANUAL:
                driveChassis();
                shootTarget();
                moveWobble();
                runIntake();
                if (gamepad1.dpad_down){
                    positionSet = true;
                    shootingLocation = poseEstimate;
                }
                if (gamepad1.a && positionSet){
//                    if (!positionSet){
//                        shootingLocation = new Pose2d(0, targets[targetInt].getY(), Math.toRadians(0));
//                    }
                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .splineTo(shootingLocation.vec(), shootingLocation.getHeading())
                            .build();
                    drive.followTrajectoryAsync(traj1);
                    driveState = DriveState.AUTOMATIC;
                }
                if (gamepad1.dpad_up){
                    driveState = DriveState.ADJUST_WOBBLE;
                }
                telemetry.addData("Status", "Manual Drive");
                break;
            case AUTOMATIC:
                // If x is pressed, we break out of the automatic following
                if (gamepad1.x) {
                    drive.cancelFollowing();
                    driveState = DriveState.MANUAL;
                }
                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    driveState = DriveState.MANUAL;
                }
//                rb.flywheelMotor.setVelocity(rb.getLaunchPower(drive.follower.getTrajectory().end(), currentTarget));
                rb.hopperUp();

                telemetry.addData("Status","Automatic Movement");
                break;
            case ADJUST_WOBBLE:
                rb.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad1.right_bumper){
                    rb.wobbleMotor.setPower(-.1);
                }
                else if (gamepad1.left_bumper){
                    rb.wobbleMotor.setPower(.1);
                } else {
                    rb.wobbleMotor.setPower(0);
                }
                if (gamepad1.a){
                    rb.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rb.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (gamepad1.b){
                    driveState = DriveState.MANUAL;
                }

        }
        //drive needs to be updated every loop. because it deals with the odometry / localization
        drive.update();
        telemetry.update();
    }


    private void shootTarget() {
        //This code cycles through the target list using the dpad
        if (gamepad1.dpad_left) {
            if (!dpadLWasDown) {
                dpadLWasDown = true;
                targetInt -= 1;
            }
        } else {
            dpadLWasDown = false;
        }
        if (gamepad1.dpad_right) {
            if (!dpadRWasDown) {
                dpadRWasDown = true;
                targetInt += 1;
            }
        } else {
            dpadRWasDown = false;
        }
        //Makes the target list wrap around.
//        if (targetInt < 0){
//            targetInt = targets.length - 1;
//        } else if (targetInt > targets.length - 1){
//            targetInt = 0;
//        }
        if (gamepad1.dpad_right){
             currentTarget = Constants.blueTopGoal;
        } else if (gamepad1.dpad_left){
            currentTarget = Constants.bluePowershot1;
        }
        Pose2d currentPosition = drive.getPoseEstimate();
//        telemetry.addData("Current Target:", currentTarget);
        //This tells the command whether or not it's the first time the button has been pressed.
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (!triggerWasDown) {
                triggerWasDown = true;
//                if (isAutoAim){
//                    double angleDifference = currentPosition.getHeading() - rb.getAngleToTarget(currentPosition, currentTarget);
//                    if (Math.abs(angleDifference) > ANGLE_TOLERANCE) {
//                        drive.turnAsync(angleDifference);
//                        driveState = DriveState.AUTOMATIC;
//                    }
//                }
                rb.getPowerAndShoot(currentPosition, currentTarget, elapsedTime, true);
            } else {
                rb.getPowerAndShoot(currentPosition, currentTarget, elapsedTime, false);
            }
        } else {
            if (triggerWasDown){
                rb.hopperDown();
            }
            rb.hammerIn();
            rb.stopFlywheel();
            triggerWasDown = false;
        }
    }

    private void driveChassis () {
        float leftStickY = -gamepad1.left_stick_y;
        float leftStickX = gamepad1.left_stick_x;
        float rightStickX = gamepad1.right_stick_x;

        double pow;
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            pow = DRIVE_POWER_SLOW;
        } else {
            pow = DRIVE_POWER;
        }
        if (Math.abs(leftStickX) + Math.abs(leftStickY) >= DRIVE_STICK_THRESHOLD || Math.abs(rightStickX) >= DRIVE_STICK_THRESHOLD) {
            rb.drive(leftStickX, leftStickY, rightStickX, pow);
        } else {
            rb.driveStop();
        }
    }

    private void moveWobble(){
        if (gamepad1.b){
            if (!bWasDown){
                bWasDown = true;
                int currentPosition = rb.wobbleMotor.getCurrentPosition();
                if (Math.abs(currentPosition - WOBBLE_GOAL_UP) > Math.abs(currentPosition - WOBBLE_GOAL_DOWN)){
                    rb.wobbleGoalUp();
                } else {
                    rb.wobbleGoalDown();
                }
            }
        } else {
            bWasDown = false;
        }
    }

    private void runIntake(){
        if (gamepad1.right_bumper){
            rb.intakeMotor.setPower(1);
        } else if (gamepad1.left_bumper){
            rb.intakeMotor.setPower(-1);
        } else {
            rb.intakeMotor.setPower(0);
        }
    }
}
