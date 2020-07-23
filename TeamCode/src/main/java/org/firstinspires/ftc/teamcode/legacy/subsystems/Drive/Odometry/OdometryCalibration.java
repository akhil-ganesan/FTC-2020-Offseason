package org.firstinspires.ftc.teamcode.legacy.subsystems.Drive.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.legacy.src.Robot;

@Autonomous
@Disabled
public class OdometryCalibration extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.getDriveSubsystem().pointRotateGyro(0.3, 90);
        robot.Telemetry();
        telemetry.addLine()
                .addData("Wheel Base Separation", (2*90*
                        Math.abs(robot.getDriveSubsystem().getLeft().getCurrentPosition())
                        + (Math.abs(robot.getDriveSubsystem().getRight().getCurrentPosition()))/
                        robot.getImu().getHeading())/(Math.PI* Motor.GoBILDA_312.getTicksPerInch()));
        telemetry.addLine()
                .addData("Horizontal Encoder Offset", robot.getDriveSubsystem()
                        .getHorizontal().getCurrentPosition()/Math.toRadians(robot.getImu().getHeading()));

    }
}
