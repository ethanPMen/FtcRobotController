package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AEONRedLong extends LinearOpMode {

    public static double leftDISTANCE = AEONBlueLong.rightDISTANCE;
    public static double forwardDISTANCE = AEONBlueLong.forwardDISTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(leftDISTANCE)
                .build();

        Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(forwardDISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(leftTrajectory);
        drive.followTrajectory(forwardTrajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}