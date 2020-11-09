package org.firstinspires.ftc.teamcode;

public class Target {
    private int x;
    private TargetType targetType;

    public int getX() {
        return x;
    }

    public TargetType getTargetType() {
        return targetType;
    }

    public Target(int x, TargetType targetType) {
        this.x = x;
        this.targetType = targetType;
    }
}
