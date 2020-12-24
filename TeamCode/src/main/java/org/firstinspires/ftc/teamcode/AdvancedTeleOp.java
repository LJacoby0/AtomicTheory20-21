package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Advanced TeleOp", group = "Remote")
public class AdvancedTeleOp extends OpMode {
    private Robot rb = new Robot(telemetry);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);


    int targetInt = 0;
    //Debouncing variables
    boolean rbWasDown;
    boolean lbWasDown;
    boolean triggerWasDown;
    //Makes a list of theea targets which will be cycled through
    Target[] targets = new Target[]{Constants.redTopGoal, Constants.redPowershot1, Constants.redPowershot2, Constants.redPowershot3};

    @Override
    public void init() {
        //This needs to be set to our actual position once we get it.
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        rb.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveChassis();
        shootTarget();
        telemetry.addData("Status", "Looping");
        telemetry.update();
        myLocalizer.update();
    }

    private void shootTarget() {
        //This code cycles through the target list using the bumpers
        if (gamepad1.right_bumper) {
            if (!rbWasDown) {
                rbWasDown = true;
                targetInt += 1;
            }
        } else {
            rbWasDown = false;
        }
        if (gamepad1.left_bumper) {
            if (!lbWasDown) {
                lbWasDown = true;
                targetInt -= 1;
            }
        } else {
            lbWasDown = false;
        }
        //Makes the target list wrap around.
        if (targetInt<0){
            targetInt = targets.length-1;
        } else if (targetInt>targets.length-1){
            targetInt = 0;
        }
        Target currentTarget = targets[targetInt];
        Pose2d currentPosition = myLocalizer.getPoseEstimate();
        telemetry.addData("Current Target:", currentTarget);
        //This tells the command whether or not it's the first time the button has been pressed.
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (!triggerWasDown) {
                triggerWasDown = true;
                //This feature is HIGHLY EXPERIMENTAL, even for this opMode. Do not comment it back in unless you know what you are doing.
//                double angleDifference = currentPosition.getHeading() - rb.getAngleToTarget(currentPosition, currentTarget);
//                drive.turnAsync(angleDifference);
                rb.aimAndShoot(currentPosition, currentTarget, elapsedTime, true);
            } else {
                rb.aimAndShoot(currentPosition, currentTarget, elapsedTime, false);
            }
        } else {
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
}
