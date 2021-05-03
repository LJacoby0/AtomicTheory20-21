package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

@Config
@TeleOp(name = "Flywheel PID Tuner", group = "Remote")
public class FlywheelPIDTuner extends LinearOpMode {
    private Robot rb = new Robot(telemetry);
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    NanoClock clock = NanoClock.system();
    public static double EFFICIENCY = .7;
    static final double TICKS_PER_ROTATION = 28;
    static final double MAX_RPM = 6000;
    static double REAL_RPM = MAX_RPM * EFFICIENCY;
    static double ROTATIONS_PER_SEC = REAL_RPM/60;
    public static double TARGET_VELOCITY = TICKS_PER_ROTATION * ROTATIONS_PER_SEC;
    public static PIDFCoefficients PID = new PIDFCoefficients(0, 0, 0, 0);
    ;
    double velocity = 0;
    public static double p;
    public static double i;
    public static double d;
    public static double f;
    double lastP;
    double lastI;
    double lastD;
    double lastF;
    @Override
    public void runOpMode() {
        rb.init(hardwareMap);
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        rb.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        PID = rb.flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            velocity = rb.flywheelMotor.getVelocity();
            if (gamepad1.a){
                rb.flywheelMotor.setVelocity(TARGET_VELOCITY);
            }
            if (gamepad1.b){
                rb.stopFlywheel();
            }
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Measured Velocity", velocity);
            telemetry.addData("error", TARGET_VELOCITY - velocity);
            if (p != lastP || i != lastI || d != lastD || f != lastF) {
                rb.flywheelMotor.setVelocityPIDFCoefficients(p, i, d, f);
                lastP = p;
                lastI = i;
                lastD = d;
                lastF = f;
            }
            telemetry.update();
        }
    }
}