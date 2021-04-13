package org.firstinspires.ftc.teamcode;

public class Target {
    private double y;
    private TargetType targetType;

    public double getY() {
        return y;
    }

    public TargetType getTargetType() {
        return targetType;
    }

    public Target(double y, TargetType targetType) {
        this.y = y;
        this.targetType = targetType;
    }
}
