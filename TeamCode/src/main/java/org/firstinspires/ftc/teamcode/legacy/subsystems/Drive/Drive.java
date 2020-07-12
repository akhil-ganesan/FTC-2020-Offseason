package org.firstinspires.ftc.teamcode.legacy.subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.legacy.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.legacy.src.Constants;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.IMU;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.KLNavX;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Vision.Vision;

import java.util.Arrays;

public class Drive extends Subsystem {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx[] driveMotors;
    private IMU imu;
    private REV_IMU imu2;
    private Vision vision;

    public Drive(IMU imu, Vision vision) {
        setImu(imu);
        setVision(vision);
    }

    public Drive(KLNavX navX, REV_IMU imu, Vision vision) {
        setImu(navX);
        setImu2(imu);
        setVision(vision);
    }

    @Override
    public void init(HardwareMap ahMap) {
        frontLeft = ahMap.get(DcMotorEx.class, Constants.frontLeft);
        frontRight = ahMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = ahMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = ahMap.get(DcMotorEx.class, Constants.backRight);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : driveMotors) {
            motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    /*@Override
    public StateMachine getStateMachine() {
        return null;
    }

     */

    // Autonomous Algorithms

    // Setting Drivetrain

    /**
     * Sets Drive to go forward/backwards
     * @param power Speed of movement
     */
    private void setDriveMotors(double power) {
        for (DcMotorEx i : driveMotors) {
            i.setPower(power);
        }
    }

    /**
     * Sets Drive to go left/right
     * @param power Speed of movement
     */
    private void setStrafeMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    /**
     * Sets Drive to Rotate
     * @param power Speed of Movement
     */
    private void setRotateMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    // Distanced Driving (Non-Motion Profiling)

    /**
     * Sets drive motors to move (forward/backwards) to a certain position
     * @param power Speed of movement
     * @param distance Distance to move (references to a target point)
     */
    public void encoderDrive(double power, int distance) {

        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotorEx i : driveMotors) {
            i.setTargetPosition(distance);
        }
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        setDriveMotors(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy());

        setDriveMotors(0);
    }

    /**
     * Sets drive motors to move (left/right) to a certain position
     * @param power Speed of movement
     * @param distance Distance to move (references to a target point)
     */
    public void encoderStrafe(double power, int distance) {

        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);

        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        setStrafeMotors(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy());

        setDriveMotors(0);
    }

    /**
     * Sets drive motors to rotate to a certain angle - (-180, 180) range
     * @param power Speed of movement
     * @param heading rotation angle
     */
    public void rotateGyro(double power, double heading) {
        if (heading < 0) {
            while (imu.getHeading() < heading) {
                setRotateMotors(-power);
            }
            setDriveMotors(0);
        } else if (heading > 0) {
            while (imu.getHeading() > heading) {
                setRotateMotors(power);
            }
            setDriveMotors(0);
        }
    }

    // Distanced Driving (Motion Profiling)

    public void motionProfileDrive(double distance) {
        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(distance);
        // Loop/Application Protocol
    }

    // TeleOp Methods

    /**
     * Tank Drive Control
     * @param left left side power
     * @param right right side power
     */
    public void tankDrive(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void mecanumDrive(double y, double x, double turn, DriveMode mode) {

        double v1 = -(y - (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v2 = -(y - (turn * Constants.strafeScaling) + (x/Constants.turnScaling));
        double v3 = -(y + (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v4 = -(y + (turn * Constants.strafeScaling) + (x/Constants.turnScaling));

        Double[] v = new Double[]{Math.abs(v1), Math.abs(v2), Math.abs(v3), Math.abs(v4)};

        Arrays.sort(v);

        if (v[3] > 1) {
            v1 /= v[3];
            v2 /= v[3];
            v3 /= v[3];
            v4 /= v[3];
        }

        frontLeft.setPower(v1 * mode.getScaling());
        backLeft.setPower(v2 * mode.getScaling());
        backRight.setPower(v3 * mode.getScaling());
        frontRight.setPower(v4 * mode.getScaling());
    }

    /**
     * Mecanum Drive Control (Via trigonometric proportions)
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Rotational Force (GamePad Right Stick x)
     * @param z Left/Right (Strafe) Force (GamePad Left Stick x)
     */
    public void trigMecDrive(double y, double x, double z) {
        double r = Math.hypot(z, y);
        double robotAngle = Math.atan2(y, z) - Math.PI/4;
        double rightX = x*-1;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1);
        backLeft.setPower(v2);
        backRight.setPower(v3);
        frontRight.setPower(v4);
    }

    /**
     * Field-Centric Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void fieldCentricMecanumDrive(double y, double x, double turn, DriveMode mode) {
        x = x * Math.cos(imu.getHeading()) - y * Math.sin(imu.getHeading());
        y = x * Math.sin(imu.getHeading()) + y * Math.cos(imu.getHeading());

        mecanumDrive(y, x, turn, mode);
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public Vision getVision() {
        return vision;
    }

    public void setVision(Vision vision) {
        this.vision = vision;
    }

    public void setImu2(REV_IMU imu2) {
        this.imu2 = imu2;
    }
}
