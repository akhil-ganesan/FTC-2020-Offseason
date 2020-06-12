package org.firstinspires.ftc.teamcode.legacy.programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.legacy.src.GameState;
import org.firstinspires.ftc.teamcode.legacy.src.Robot;
import org.firstinspires.ftc.teamcode.legacy.subsystems.settings.DriveMode;

@TeleOp(name = "Test Chassis", group = "Test")
public class Test_Chassis extends OpMode {
    Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.resetElapsedTime();

        if (robot.getGameState() == GameState.TeleOp) {
            robot.getDriveSubsystem().mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, DriveMode.Normal);
        } else {
            robot.getDriveSubsystem().mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, DriveMode.Economy);
        }

    }
}
