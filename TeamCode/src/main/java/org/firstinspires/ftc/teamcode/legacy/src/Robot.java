package org.firstinspires.ftc.teamcode.legacy.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.KLAHRS;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.KLNavXBasic;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Vision.Vuforia;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private GameState gameState;

    private Subsystem[] subsystems;

    private REV_IMU imu = new REV_IMU();
    //private KLAHRS navX = new KLAHRS();
    private Vuforia vuforia = new Vuforia();
    //private Drive DriveSubsystem = new Drive(navX, imu, vuforia);
    private Drive DriveSubsystem = new Drive(imu, vuforia);

    public Robot(HardwareMap hMap, Telemetry tele) {
        hardwareMap = hMap;
        telemetry = tele;
    }

    public void init() {
        subsystems = new Subsystem[]{DriveSubsystem, imu};

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
                .addData("Robot Initialized:", true);
        // Game State
        telemetry.addLine()
                .addData("Game State", getGameState().getName());
        // IMU Measurements
        telemetry.addLine()
                .addData("Roll", getImu().getRoll())
                .addData("Pitch", getImu().getPitch())
                .addData("Heading", getImu().getHeading());

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
}
