package org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.team18103.src.Robot;

@Autonomous
@Disabled
public class OdometryCalibration extends LinearOpMode {
    Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.getDriveSubsystem().PointRotation(90);
        robot.Telemetry();
        telemetry.addLine()
                .addData("Wheel Base Separation", (2*90*
                        Math.abs(robot.getDriveSubsystem().getOdometry().getLeft().getCurrentPosition())
                        + (Math.abs(robot.getDriveSubsystem().getOdometry().getRight().getCurrentPosition()))/
                        robot.getDriveSubsystem().getDataFusionTheta())/(Math.PI* Motor.GoBILDA_312.getTicksPerInch()));
        telemetry.addLine()
                .addData("Horizontal Encoder Offset", robot.getDriveSubsystem().getOdometry()
                        .getHorizontal().getCurrentPosition()/Math.toRadians(robot.getDriveSubsystem().getDataFusionTheta()));

    }
}
