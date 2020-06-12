package org.firstinspires.ftc.teamcode.legacy.subsystems.settings;

public enum DriveMode {
    Sport(1),
    Normal(0.5),
    Economy(0.25);

    private final double scaling;

    DriveMode(double scaling) {
        this.scaling = scaling;
    }

    public double getScaling() {
        return scaling;
    }
}
