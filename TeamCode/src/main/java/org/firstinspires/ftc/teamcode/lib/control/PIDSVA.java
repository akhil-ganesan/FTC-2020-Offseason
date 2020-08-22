package org.firstinspires.ftc.teamcode.lib.control;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.Dt;

public class PIDSVA {
    double kp, ki, kd, ks, kv, ka;
    double prev_error, integral, derivative;

    public PIDSVA(double kp, double ki, double kd, double ks, double kv, double ka) {
        setConstants(kp, ki, kd, ks, kv, ka);
    }

    public double getOutput(double error, double velocity, double acceleration) {
        double kf = ks + (velocity * kv) + (acceleration * ka);
        integral += error * Dt;
        derivative = (error - prev_error)/Dt - velocity;
        prev_error = error;
        return (kp * error) + (ki * integral) + (kd * derivative) + kf;
    }

    public void setConstants(double kp, double ki, double kd, double ks, double kv, double ka) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        prev_error = 0;
        integral = 0;
    }
}
