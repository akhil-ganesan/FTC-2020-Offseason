package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.Odometry;

@Deprecated
public class MecanumOdometryGPS extends Odometry {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double ticksPerInch, dt;

    private double x = 0, y = 0, theta = 0;
    private double r_0 = 0, l_0 = 0, s_0 = 0;

    public MecanumOdometryGPS(double ticksPerInch, int dt) {
        this.ticksPerInch = ticksPerInch;
        this.dt = dt;
    }

    public MecanumOdometryGPS(double ticksPerInch, int dt, double x0, double y0, double theta0) {
        this.ticksPerInch = ticksPerInch;
        this.dt = dt;
        x = x0;
        y = y0;
        theta = theta0;
    }

    @Override
    public void init(HardwareMap ahMap) {
        frontLeft = ahMap.get(DcMotorEx.class, Constants.frontLeft);
        frontRight = ahMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = ahMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = ahMap.get(DcMotorEx.class, Constants.backRight);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void run() {
        //Get Current Positions
        double lPos = (getLeft() * getTicksPerInch());
        double rPos = (getRight() * getTicksPerInch());

        double dl = lPos - getL_0();
        double dr = rPos - getR_0();

        //Calculate Angle
        double dTheta = (dl - dr) / (Constants.ENCODER_DIFFERENCE);
        theta += dTheta;

        //Get the components of the motion
        double sPose = (getHorizontal() * getTicksPerInch());
        double ds = (sPose - s_0) - (dTheta *Constants.HORIZONTAL_OFFSET);

        double p = ((dr + dl) / (2 * getTheta()));

        //Calculate and update the position values
        double dx = p * Math.sin(dTheta);
        double dy = -dx * Math.tan(dTheta /2) + ds * Math.cos(dTheta /2);
        dx += ds * Math.sin(dTheta /2);

        x += dx;
        y += dy;

        setZeros(lPos, rPos, sPose);
    }

    @Override
    public double getX() {
        run();
        return x;
    }

    @Override
    public double getY() {
        run();
        return y;
    }

    @Override
    public double getTheta() {
        run();
        return theta;
    }

    public int getHorizontal() {
        return ((backLeft.getCurrentPosition() + frontRight.getCurrentPosition()) -
                (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()))/4;
    }

    public int getLeft() {
        return (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2;
    }

    public int getRight() {
        return (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getL_0() {
        return l_0;
    }

    public double getR_0() {
        return r_0;
    }

    public double getS_0() {
        return s_0;
    }

    public void setZeros(double l_0, double r_0, double s_0) {
        this.l_0 = l_0;
        this.r_0 = r_0;
        this.s_0 = s_0;
    }

}
