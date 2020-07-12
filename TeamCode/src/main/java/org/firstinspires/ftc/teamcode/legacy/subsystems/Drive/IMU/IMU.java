package org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.legacy.subsystems.Subsystem;

public abstract class IMU extends Subsystem {

    public abstract double getHeading();

    public abstract double getRoll();

    public abstract double getPitch();


}
