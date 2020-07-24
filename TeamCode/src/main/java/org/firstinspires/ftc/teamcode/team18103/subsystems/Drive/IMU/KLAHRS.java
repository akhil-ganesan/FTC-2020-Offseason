package org.firstinspires.ftc.teamcode.team18103.subsystems.Drive.IMU;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.*;

public class KLAHRS extends IMU {
    private AHRS NavX;
    private double last_world_linear_accel_x = 0.0;
    private double last_world_linear_accel_y = 0.0;

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
        boolean collisionDetected = false;

        double curr_world_linear_accel_x = NavX.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = NavX.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
            collisionDetected = true;
        }
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
