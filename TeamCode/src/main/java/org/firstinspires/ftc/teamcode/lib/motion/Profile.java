package org.firstinspires.ftc.teamcode.lib.motion;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class Profile {
    public abstract Double[] generateProfile();

    public void run(Double[] velocities, DcMotorEx[] drivers) {
        for (double i : velocities) {
            for (DcMotorEx motor : drivers) {
                motor.setVelocity(i);
            }
        }
    }

    public abstract double getDecelerationDist();

}
