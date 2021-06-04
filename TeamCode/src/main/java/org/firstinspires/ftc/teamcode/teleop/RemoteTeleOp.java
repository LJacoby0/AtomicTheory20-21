package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.kotlin.extensions.util.InterpLUTExtKt;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.UGAdvancedHighGoalPipeline;
import org.firstinspires.ftc.teamcode.vision.UGAngleHighGoalPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.util.SolveSimultaneousEquations.*;

public abstract class RemoteTeleOp extends OpMode {
    public abstract UGAngleHighGoalPipeline.Target getColor();
    private Robot rb = new Robot(telemetry);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    UGAdvancedHighGoalPipeline pipeline = new UGAdvancedHighGoalPipeline(105, 10);
//    double currentTurretDegrees = 30;
    public enum ShootState {
        SHOOTING,
        WINDUP,
        IDLE
    }
    public enum DriveState {
        MANUAL,
        AUTOMATIC,
//        ADJUST_WOBBLE
    }

    DriveState driveState = DriveState.MANUAL;
    ShootState shootState = ShootState.IDLE;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        });
        drive = new SampleMecanumDrive(hardwareMap);
        rb.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (driveState){
            case MANUAL:
                driveChassis();
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
        }
        shootTarget();
        runIntake();
        moveWobble();
        telemetry.addData("Distance from target", pipeline.getDistanceToGoal(getColor()));
        telemetry.addData("Angle to target", pipeline.calculateYaw(getColor()));
        telemetry.addData("Status", "Looping");
        telemetry.update();
        drive.update();
    }

    private void moveWobble(){
        if (gamepad1.a){
            rb.wobbleGoalUp();
        }
        if(gamepad1.b){
            rb.wobbleGoalDown();
        }
    }
    double shootStartTime = 0;
    double timeSinceStart;
    UGAdvancedHighGoalPipeline.Powershot powershot = null;
    boolean yWasDown = false;
    //This is purposely very slow, so if something goes wrong it's easy to troubleshoot
    double targetVelocity = 200;

    private void shootTarget() {
        if (gamepad1.y){
            if (!yWasDown){
                switch (powershot){
                    case LeftShot:
                        powershot = UGAdvancedHighGoalPipeline.Powershot.RightShot;
                    case RightShot:
                        powershot = UGAdvancedHighGoalPipeline.Powershot.CenterShot;
                    case CenterShot:
                        powershot = null;
                    default:
                        powershot = UGAdvancedHighGoalPipeline.Powershot.LeftShot;
                }
                yWasDown = true;
            }
        } else {
            yWasDown = false;
        }
        switch (shootState){
            //It starts off IDLE, but if the right trigger is pressed it enters WINDUP
            case IDLE:
                rb.hopperDown();
                rb.flywheelMotor.setVelocity(0);
                if (gamepad1.right_trigger > TRIGGER_THRESHOLD){
                    aimAtTarget();
                    if (powershot == null){
                        targetVelocity = rb.getLaunchPower(pipeline.getDistanceToGoal(getColor()), false);
                    } else {
                        targetVelocity = rb.getLaunchPower(pipeline.getPowerShotDistance(getColor(), powershot), true);
                    }
                    rb.flywheelMotor.setVelocity(targetVelocity);
                    shootState = ShootState.WINDUP;
                }
                break;
            //In WINDUP, it waits for the velocity of the wheel to be correct,
            //Then goes to SHOOTING. However, it goes back to IDLE, cancelling the operation,
            //if x is pressed. 40 is the current tolerance, it might need to be bigger.
            case WINDUP:
                rb.hopperUp();
                if (Math.abs(rb.flywheelMotor.getVelocity() - targetVelocity) < 60){
                    shootStartTime = elapsedTime.time();
                    shootState = ShootState.SHOOTING;
                }
                if (gamepad1.x){
                    shootState = ShootState.IDLE;
                }
                break;
            //In SHOOTING, it pushes the ring forward until finished, then withdraws the hammer.
            //Once that's done, it goes back to IDLE
            case SHOOTING:
                timeSinceStart = elapsedTime.time() - shootStartTime;
                if (timeSinceStart < HAMMER_SERVO_ROTATION_TIME) {
                    rb.hammerPush();
                } else if (timeSinceStart < DOUBLE_HAMMER_SERVO_ROTATION_TIME_MILLISECONDS){
                    rb.hammerBack();
                } else {
                    shootState = ShootState.IDLE;
                }
                break;
        }
        telemetry.addData("Shoot State", shootState);
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
    private void aimAtTarget(){
        double degreesToTarget = pipeline.calculateYaw(getColor());
        drive.turnAsync(Math.toRadians(degreesToTarget));
        driveState = DriveState.AUTOMATIC;
        //This solves the simultaneous equation: maxRobotAngVelocity*degreesRobotMoves = maxTurretAngVelocity*degreesTurretMoves
        //and degreesTurretMoves+degreesRobotMoves = degreesToTarget

//        double[] movements = solveSimultaneousEquations(DriveConstants.MAX_ANG_VEL_DEG, -TURRET_ROTATION_SPEED, 1, 1, 0 , degreesToTarget);
//        double robotRotationDegrees = movements[0];
//        double turretRotationDegrees = movements[1];
//        double impossibleTurretDegrees = 0;
//        if (currentTurretDegrees + turretRotationDegrees > TURRET_MAX_ROTATiON_DEGREES){
//            impossibleTurretDegrees = currentTurretDegrees + turretRotationDegrees - TURRET_MAX_ROTATiON_DEGREES;
//        } else if (currentTurretDegrees - turretRotationDegrees < TURRET_MIN_ROTATiON_DEGREES){
//            impossibleTurretDegrees =  currentTurretDegrees - turretRotationDegrees + TURRET_MIN_ROTATiON_DEGREES;
//        }
//        turretRotationDegrees -= impossibleTurretDegrees;
//        robotRotationDegrees += impossibleTurretDegrees;
//        drive.turnAsync(Math.toRadians(robotRotationDegrees));
//        rb.rotateTurretDegrees(turretRotationDegrees);
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
}
