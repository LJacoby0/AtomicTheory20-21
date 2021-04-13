package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odometry System Hardware Check", group = "Calibration")
public class OdometryHardwareTest extends OpMode {
    private Robot rb = new Robot(telemetry);
    @Override
    public void init() {
        rb.init(hardwareMap);
        rb.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.addData("Left Encoder Value", rb.leftEncoder.getCurrentPosition());
        telemetry.addData("Horizontal Encoder Value", rb.middleEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Value", rb.rightEncoder.getCurrentPosition());
        telemetry.addData("flywheel Encoder Value", rb.flywheelMotor.getCurrentPosition());
        telemetry.addData("intake Encoder Value", rb.intakeMotor.getCurrentPosition());
        telemetry.addData("bakRight Encoder Value", rb.backRightMotor.getCurrentPosition());
        telemetry.addData("bl Encoder Value", rb.backLeftMotor.getCurrentPosition());
        telemetry.addData("fl Encoder Value", rb.frontLeftMotor.getCurrentPosition());
        telemetry.addData("fr Encoder Value", rb.frontRightMotor.getCurrentPosition());
        telemetry.addData("Wobble Encoder Value", rb.wobbleMotor.getCurrentPosition());



        telemetry.update();

    }
}
