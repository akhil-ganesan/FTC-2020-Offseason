package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.ProfileState;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.DiWheelOdometryGPS;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.VuforiaVision;

import java.util.Arrays;

@Deprecated
public class Drive_2 extends Subsystem {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx[] driveMotors;
    private DriveMode driveMode = DriveMode.Balanced;
    private int driveType = 0; // 0 - Field-Centric, 1 - POV
    private IMU imu;
    private DiWheelOdometryGPS odometry;
    private VuforiaVision vision;
    private ProfileState driveState;

    public Drive_2(IMU imu) {
        this.imu = imu;
    }

    public Drive_2(IMU imu, DiWheelOdometryGPS odometry) {
        this.imu = imu;
        this.odometry = odometry;
    }

    public Drive_2(IMU imu, VuforiaVision vision) {
        this.imu = imu;
        this.vision = vision;
    }

    public Drive_2(IMU imu, DiWheelOdometryGPS odometry, VuforiaVision vision) {
        this.imu = imu;
        this.odometry = odometry;
        this.vision = vision;
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
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

    }

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
    public void IMUPointRotateGyro(double power, double heading) {
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
        TrapezoidalMotionProfileGenerator motionProfile = new TrapezoidalMotionProfileGenerator(distance, Motor.GoBILDA_312);
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime();
        PIDSVA controller = new PIDSVA(10d/12d, 1d/12d, 10d/12d, 0d, 1d/motionProfile.getMaxV(), 0.01d/12d);
        while (timer.seconds() < motionProfile.getTotalTime()) {
            double timeStamp = timer.seconds();
            double position = motionProfile.getPosition(timeStamp);
            double velocity = motionProfile.getVelocity(timeStamp);
            double acceleration = motionProfile.getAcceleration(timeStamp);
            double error = position - (frontLeft.getCurrentPosition()/ Motor.GoBILDA_312.getTicksPerInch());
            double output = controller.getOutput(error, velocity, acceleration);

            for (DcMotorEx i : driveMotors) {
                i.setPower(output);
            }

            setDriveState(motionProfile.getProfileState(timeStamp));

        }
        setDriveMotors(0);
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
    public void POVMecanumDrive(double y, double x, double turn, DriveMode mode) {

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

        POVMecanumDrive(y, x, turn, mode);
    }

    /**
     * Ultimate Drive Controller
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param left_trigger Lower Gear Adjustment
     * @param right_trigger Higher Gear Adjustment
     * @param left_bumper Field-Centric Drive Setter
     * @param right_bumper Point-of-View Drive Setter
     */
    public void ultimateDriveController(double y, double x, double turn, float left_trigger,
                                        float right_trigger, boolean left_bumper, boolean right_bumper) {
        double mode = driveMode.getId();
        double modeChange = right_trigger - left_trigger;
        if (Math.abs(modeChange) > 0.5) {
            mode = MathFx.scale(-1, 1, Math.round(mode + modeChange));
        }

        setDriveMode(getDMbyID(mode));

        if (left_bumper) {
            setDriveType(0);
        }

        if (right_bumper) {
            setDriveType(1);
        }

        switch (driveType) {
            case 0:
                fieldCentricMecanumDrive(y, x, turn, driveMode);
                break;
            case 1:
                POVMecanumDrive(y, x, turn, driveMode);
                break;
        }


    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public DriveMode getDMbyID(double id) {
        if (id == -1) {
            return DriveMode.Economy;
        } else if (id == 0) {
            return DriveMode.Balanced;
        } else {
            return DriveMode.Sport;
        }
    }

    public double getDriveType() {
        return driveType;
    }

    public void setDriveType(int driveType) {
        this.driveType = driveType;
    }

    public DiWheelOdometryGPS getOdometry() {
        return odometry;
    }

    public VuforiaVision getVision() {
        return vision;
    }

    public void setDriveState(ProfileState driveState) {
        this.driveState = driveState;
    }

    public ProfileState getDriveState() {
        return driveState;
    }
}
