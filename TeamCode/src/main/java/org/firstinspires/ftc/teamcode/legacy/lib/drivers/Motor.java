package org.firstinspires.ftc.teamcode.legacy.lib.drivers;

public enum Motor {
    GoBILDA_1150(1150d, 145.6d),
    GoBILDA_435(435d, 383.6d),
    GoBILDA_312(312d, 537.6d),
    GoBILDA_223(223d, 753.2d),
    NeveRest_3_7(1780d, 103.6d),
    REV_Core_Hex(125d, 288d);

    private final double RPM;
    private final double ENCODER_TICKS_PER_REVOLUTION;

    Motor(double RPM, double encoderTicksPerRevolution) {
        this.RPM = RPM;
        this.ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return RPM;
    }

    public double getENCODER_TICKS_PER_REVOLUTION() {
        return ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getTicksPerInch() {
        return getENCODER_TICKS_PER_REVOLUTION()/3.937;
    }
}
