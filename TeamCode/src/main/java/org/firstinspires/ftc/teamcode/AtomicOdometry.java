package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class AtomicOdometry {
    public static final double COUNTS_PER_INCH = 32.362461;
    private static final double ROTATION_CONSTANT = 1;

    private double x = 0;
    private double y = 0;
    private double theta;

    public double oldLeftPosition = 0;
    public double oldRightPosition = 0;
    public double oldHorizontalPosition = 0;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void update(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder) {
        double leftChange = verticalEncoderLeft.getCurrentPosition() - oldLeftPosition;
        double rightChange = verticalEncoderRight.getCurrentPosition() - oldRightPosition;
        double horizontalChange = horizontalEncoder.getCurrentPosition() - oldHorizontalPosition;


        double dx = 0;
        double dy = 0;
        double dTheta = 0;

        double changeInRobotOrientation = (leftChange - rightChange) / ROTATION_CONSTANT;
        oldLeftPosition = verticalEncoderLeft.getCurrentPosition();
        oldRightPosition = verticalEncoderRight.getCurrentPosition();
        oldHorizontalPosition = horizontalEncoder.getCurrentPosition();

        x += dx;
        y += dy;
        theta += dTheta;
    }

}
