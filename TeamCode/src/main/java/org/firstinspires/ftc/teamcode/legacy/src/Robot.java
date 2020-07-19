package org.firstinspires.ftc.teamcode.legacy.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.legacy.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.Odometry.OdometryGPS;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Vision.Vuforia;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private GameState gameState;

    private Subsystem[] subsystems;

    private REV_IMU imu = new REV_IMU();
    //private AHRS navX = new KLAHRS();
    private Vuforia vuforia = new Vuforia();
    private OdometryGPS odometry = new OdometryGPS(Motor.GoBILDA_312.getTicksPerInch(), Constants.dt);
    //private Drive DriveSubsystem = new Drive(navX, imu, vuforia);
    //private Drive DriveSubsystem = new Drive(imu, vuforia);
    private Drive DriveSubsystem = new Drive(imu, vuforia, odometry);

    public Robot(HardwareMap hMap, Telemetry tele) {
        hardwareMap = hMap;
        telemetry = tele;
    }

    public void init() {
        subsystems = new Subsystem[]{DriveSubsystem, imu, vuforia, odometry};

        for (Subsystem subsystem : subsystems) {
            subsystem.init(hardwareMap);
        }

        setGameState(GameState.TeleOp);

        resetElapsedTime();

        Telemetry();
    }

    public void Telemetry() {
        // Robot Init
        telemetry.addLine()
                .addData("Robot Initialized: ", true);
        // Game State
        telemetry.addLine()
                .addData("Game State: ", getGameState().getName());
        // IMU Measurements
        telemetry.addLine()
                .addData("IMU Roll: ", getImu().getRoll())
                .addData("IMU Pitch: ", getImu().getPitch())
                .addData("IMU Heading: ", getImu().getHeading());
        // Odometry
        telemetry.addLine()
                .addData("X: ", getOdometry().getX())
                .addData("Y: ", getOdometry().getY())
                .addData("Theta: ", getOdometry().getTheta());
        // Collision Detection
        telemetry.addLine()
                .addData("Collision Detected: ", getImu().getCollision());
        // Drive Mode
        telemetry.addLine()
                .addData("Drive Mode: ", getDriveSubsystem().getDriveMode().getName());

        telemetry.update();
    }

    public GameState getGameState() {
        if (elapsedTime.seconds() < GameState.TeleOp.getEndTime()) {
            return GameState.TeleOp;
        } else {
            return GameState.EndGame;
        }
    }

    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    public Drive getDriveSubsystem() {
        return DriveSubsystem;
    }

    public REV_IMU getImu() {
        return imu;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }

    public OdometryGPS getOdometry() {
        return odometry;
    }
}
