package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Remote TeleOp", group = "Remote")
public class RemoteTeleOp extends OpMode {
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
                rb.shoot(Constants.FLYWHEEL_CONSTANT, elapsedTime, true, isManual);
            }
                rb.shoot(Constants.FLYWHEEL_CONSTANT, elapsedTime, false, isManual);
        } else {
            triggerWasDown = false;
            rb.stopFlywheel();
            //If it's in automatic mode, the hopper needs to be told to go down while it's not pressed.
            if (!isManual){
                rb.hopperDown();
                rb.hammerIn();
            }
        }
    }
    private void runIntake(){
        if (gamepad1.right_bumper){
            rb.intakeMotor.setPower(1);
        } else if (gamepad1.left_bumper){
            rb.intakeMotor.setPower(-1);
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
            if(gamepad1.right_bumper) {
                isManual = true;
            } else if (gamepad1.left_bumper){
                isManual = false;
            }
            if (isManual){
                if(gamepad1.x) {
                    rb.hopperDown();
                } else if(gamepad1.y) {
                    rb.hopperUp();
                }
                if(gamepad1.right_bumper) {
                    rb.hammerIn();
                } else if(gamepad1.left_bumper) {
                    rb.hammerOut();
                }
            }
        }
    }
