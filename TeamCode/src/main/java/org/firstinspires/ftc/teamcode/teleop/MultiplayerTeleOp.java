package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name = "Two Player TeleOp", group = "Remote")
public class MultiplayerTeleOp extends OpMode {
    private Robot rb = new Robot(telemetry);
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static boolean isManual = false;

    @Override
    public void init() {
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
        moveHopper();
        runIntake();
        telemetry.addData("Status", "Looping");
        telemetry.addData("Manual Mode", isManual);
        telemetry.update();
    }

    static boolean triggerWasDown;

    private void shootTarget() {
        //This tells the command whether or not it's the first time the button has been pressed.
        //It also passes in whether or not we are in manual mode.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            if (!triggerWasDown) {
                triggerWasDown = true;
                rb.shoot(FLYWHEEL_MAX_VELO, elapsedTime, true);
            }
                rb.shoot(FLYWHEEL_MAX_VELO, elapsedTime, false);
        } else {
            triggerWasDown = false;
            rb.stopFlywheel();
            //If it's in automatic mode, the hopper needs to be told to go down while it's not pressed.
            if (!isManual){
                rb.hopperDown();
                rb.hammerBack();
            }
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

        //had to put on different buttons because of debouncing (thought of repeated button press)
        private void moveHopper(){
            if(gamepad2.dpad_down) {
                isManual = true;
            } else if (gamepad2.dpad_up){
                isManual = false;
            }
            if (isManual){
                if(gamepad2.x) {
                    rb.hopperDown();
                } else if(gamepad2.y) {
                    rb.hopperUp();
                }
                if(gamepad2.a) {
                    rb.hammerBack();
                } else if(gamepad2.b) {
                    rb.hammerPush();
                }
            }
        }
    }
