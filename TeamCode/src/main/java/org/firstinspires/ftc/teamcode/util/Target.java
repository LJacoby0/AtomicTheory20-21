package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.NotNull;

public class Target {
    private double y;
    private TargetType targetType;

    public double getY() {
        return y;
    }

    public TargetType getTargetType() {
        return targetType;
    }

    public Target(double y, @NotNull TargetType targetType) {
        this.y = y;
        this.targetType = targetType;
    }
}
