package org.firstinspires.ftc.teamcode.team18103.subsystems.Drive.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.COLLISION_THRESHOLD_DELTA_G;

public class REV_IMU extends IMU {
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private double last_world_linear_accel_x = 0.0;
    private double last_world_linear_accel_y = 0.0;

    @Override
    public void init(HardwareMap ahMap) {
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    //@Override
    //public StateMachine getStateMachine() {
    //    return null;
    //}

    @Override
    public double getHeading() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }

    @Override
    public double getRoll() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.secondAngle;
    }

    @Override
    public double getPitch() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.thirdAngle;
    }

    @Override
    public boolean getCollision() {
        boolean collision = false;

        double curr_world_linear_accel_x = imu.getLinearAcceleration().xAccel;
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = imu.getLinearAcceleration().yAccel;
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
            collision = true;
        }

        return collision;
    }

    public double getGravity() {
        gravity  = imu.getGravity();
        return Double.parseDouble(String.valueOf(gravity));
    }

    public double getMag() {
        gravity  = imu.getGravity();
        return Math.sqrt(gravity.xAccel*gravity.xAccel
                + gravity.yAccel*gravity.yAccel
                + gravity.zAccel*gravity.zAccel);
    }

    public double getTemp() {
        return Double.parseDouble(String.valueOf(imu.getTemperature()));
    }

}
