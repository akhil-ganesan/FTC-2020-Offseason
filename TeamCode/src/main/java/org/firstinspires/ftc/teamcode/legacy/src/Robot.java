package org.firstinspires.ftc.teamcode.legacy.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Drive;
import org.firstinspires.ftc.teamcode.legacy.subsystems.IMU;
import org.firstinspires.ftc.teamcode.legacy.subsystems.Subsystem;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private GameState gameState;

    private Subsystem[] subsystems;

    private IMU imu = new IMU();
    private Drive DriveSubsystem = new Drive(imu);

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

    public IMU getImu() {
        return imu;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }
}
