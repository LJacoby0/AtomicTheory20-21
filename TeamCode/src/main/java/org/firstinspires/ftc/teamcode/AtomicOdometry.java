package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class AtomicOdometry {
    public static final double COUNTS_PER_INCH = Constants.COUNTS_PER_INCH;
    private static final double ROTATION_CONSTANT = 1;
    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    //Algorithm constants
    double robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
    double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

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

    double robotOrientationRadians;
    double changeInRobotOrientation;

    public void update(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder) {
        double leftChange = verticalEncoderLeft.getCurrentPosition() - oldLeftPosition;
        double rightChange = verticalEncoderRight.getCurrentPosition() - oldRightPosition;
        double rawHorizontalChange = horizontalEncoder.getCurrentPosition() - oldHorizontalPosition;


        double dx = 0;
        double dy = 0;
        double dTheta = 0;

        changeInRobotOrientation = (leftChange - rightChange) / robotEncoderWheelDistance;
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        double horizontalChange = rawHorizontalChange;


        oldLeftPosition = verticalEncoderLeft.getCurrentPosition();
        oldRightPosition = verticalEncoderRight.getCurrentPosition();
        oldHorizontalPosition = horizontalEncoder.getCurrentPosition();

        x += dx;
        y += dy;
        theta += dTheta;
    }

}
