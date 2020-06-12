package org.firstinspires.ftc.teamcode.legacy.lib.util;

public class MathFx {

    public static double scale(double lower, double upper, double value) {
        return Math.max(lower, Math.min(value, upper));
    }

}
