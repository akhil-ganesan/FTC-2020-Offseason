package org.firstinspires.ftc.teamcode.team18103.subsystems.IMU;

import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

public abstract class IMU extends Subsystem {

    public abstract double getHeading();

    public abstract double getRoll();

    public abstract double getPitch();

    public abstract boolean getCollision();


}
