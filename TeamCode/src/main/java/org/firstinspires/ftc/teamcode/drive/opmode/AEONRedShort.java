package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Elevator;

@Config
@Autonomous(group = "drive")
public class AEONRedShort extends LinearOpMode {
    private boolean left;
    public AEONRedShort(boolean left) {
        this.left = left;
    }
    public AEONRedShort() {
        this.left = true;
    }
    public static double leftDISTANCE = 34;
    public static double forwardDISTANCE = 36;
    public static double backDISTANCE = 1;
    public static double rightDISTANCE = 32;
    public static double parkDISTANCE = 16;
    private Elevator elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        elevator = Elevator.getInstance(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(this.left ? leftDISTANCE : -leftDISTANCE)
                .build();

        Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(forwardDISTANCE)
                .build();

        Trajectory backTrajectory = drive.trajectoryBuilder(new Pose2d())
                .back(backDISTANCE)
                .build();

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(this.left ? rightDISTANCE : -rightDISTANCE)
                .build();

        Trajectory parkTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(parkDISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;
//        elevator.setTelemetry(telemetry);
        drive.followTrajectory(leftTrajectory);
        drive.followTrajectory(forwardTrajectory);
        elevator.runToPosition(15);
        elevator.openTrapDoor();
        Thread.sleep(2000);
        drive.followTrajectory(backTrajectory);
        elevator.closeTrapDoor();
        elevator.runToPosition(1);
        drive.followTrajectory(rightTrajectory);
        drive.followTrajectory(parkTrajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}