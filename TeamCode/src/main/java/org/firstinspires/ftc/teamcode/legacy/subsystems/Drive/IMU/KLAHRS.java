package org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.legacy.src.Constants.*;

public class KLAHRS extends IMU {
    private AHRS NavX;

    @Override
    public void init(HardwareMap ahMap) {
        NavX = AHRS.getInstance(ahMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
    }

    @Override
    public double getHeading() {
        return NavX.getYaw();
    }

    @Override
    public double getRoll() {
        return NavX.getRoll();
    }

    @Override
    public boolean getCollision() {
        return false;
    }

    @Override
    public double getPitch() {
        return NavX.getPitch();
    }

    public double getDx() {
        return NavX.getRawAccelX();
    }

    public double getDy() {
        return NavX.getRawAccelY();
    }

    public double getDz() {
        return NavX.getRawAccelZ();
    }

    public void stop() {
        NavX.close();
    }

}
