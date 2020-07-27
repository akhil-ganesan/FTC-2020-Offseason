package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.IMU;

@Deprecated
public class KLNavXBasic extends IMU {
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navX;
    AngularVelocity rates;
    Orientation angles;

    @Override
    public void init(HardwareMap ahMap) {
        navX = ahMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navX;

        while (navX.isCalibrating());

        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public double getHeading() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    @Override
    public double getRoll() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    @Override
    public double getPitch() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    @Override
    public boolean getCollision() {
        return false;
    }

    public double getDx() {
        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return rates.xRotationRate;
    }

    public double getDy() {
        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return rates.yRotationRate;
    }

    public double getDz() {
        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return rates.zRotationRate;
    }

}
