package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.Drawable;

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
        telemetry.addData("Status", "Looping");
        telemetry.update();
    }

    boolean triggerWasDown;

    private void shootTarget() {
        //This tells the command whether or not it's the first time the button has been pressed.
        if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            rb.flywheelMotor.setPower(Constants.FLYWHEEL_CONSTANT);
        } else {
            rb.flywheelMotor.setPower(0);
        }

//            if (!triggerWasDown) {
//                triggerWasDown = true;
//                rb.shoot(Constants.FLYWHEEL_CONSTANT, elapsedTime, true);
//            } else {
//                rb.shoot(0, elapsedTime, false);
//            }
//        } else {
//            triggerWasDown = false;
//        }
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

        boolean hammerin;
        boolean hopperup;

        //was being kind of funky, seemed to be 'thinking' so probably an issue with it reading repeated presses
        //look up "debouncing"
        private void moveHopper(){
            if(gamepad2.x){
                if(hopperup){
                    hopperup = false;
                    rb.hopper_rotate.setPosition(Constants.HOPPER_DOWN);
                } else if (!hopperup){
                    hopperup = true;
                    rb.hopper_rotate.setPosition(Constants.HOPPER_UP);
                } else{
                    telemetry.addData("panic:", "hopper");
                }
                telemetry.addData("keypressed:", "x");
            }
            if(gamepad2.y){
                if(hammerin){
                    hammerin = false;
                    rb.hopper_hammer.setPosition(Constants.HAMMER_OUT);
                } else if(!hammerin){
                    hammerin = true;
                    rb.hopper_hammer.setPosition(Constants.HAMMER_IN);
                } else{
                    telemetry.addData("panic:", "hammer");
                }
                telemetry.addData("keypressed:", "y");
            }
            telemetry.update();
        }
    }
