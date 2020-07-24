package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;

public class Base_Auto extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
    }
}
