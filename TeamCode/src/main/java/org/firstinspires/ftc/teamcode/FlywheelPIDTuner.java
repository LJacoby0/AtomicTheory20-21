package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

@TeleOp(name = "Flywheel PID Tuner", group = "Remote")
public class FlywheelPIDTuner extends LinearOpMode {
    private Robot rb = new Robot(telemetry);
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    NanoClock clock = NanoClock.system();
    private static double EFFICIENCY = .85;
    static final double TICKS_PER_ROTATION = 28;
    static final double MAX_RPM = 6000;
    static double REAL_RPM = MAX_RPM*EFFICIENCY;
    static double ROTATIONS_PER_SEC = REAL_RPM/60;
    static double TARGET_VELOCITY = TICKS_PER_ROTATION * ROTATIONS_PER_SEC;
    double velocity = rb.flywheelMotor.getVelocity();
    double lastKp = PID.p;
    double lastKi = MOTOR_VELO_PID.i;
    double lastKd = MOTOR_VELO_PID.d;
    double lastKf = MOTOR_VELO_PID.f;

    static PIDFCoefficients PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(TARGET_VELOCITY));
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        while (!isStopRequested()) {
            velocity = rb.flywheelMotor.getVelocity();
            if (gamepad1.a){
                rb.flywheelMotor.setVelocity(TARGET_VELOCITY);
            }
            telemetry.addData("targetVelocity", TARGET_VELOCITY);
            telemetry.addData("measuredVelocity", velocity);
            telemetry.addData("error", TARGET_VELOCITY - velocity);
            if (lastKp != PID.p || lastKd != PID.d
                    || lastKi != PID.i || lastKf != PID.f) {
                rb.flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PID);
                lastKp = PID.p;
                lastKi = PID.i;
                lastKd = PID.d;
                lastKf = PID.f;
            }
        }
    }
}
