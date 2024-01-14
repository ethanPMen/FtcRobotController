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
public class AEONRedLong extends LinearOpMode {

    private boolean left;
    public AEONRedLong(boolean left) {
        this.left = left;
    }
    public AEONRedLong() {
        this.left = true;
    }

    public static double leftDISTANCE = 67;
    public static double forwardDISTANCE = 71;
    public static double rightDISTANCE = 16;
    public static double adjustDISTANCE = 28;
    public static double parkDISTANCE = 36;
    public static double park2DISTANCE = 38;
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

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(this.left ? rightDISTANCE : -rightDISTANCE)
                .build();

        Trajectory adjustTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(adjustDISTANCE)
                .build();

        Trajectory parkTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(this.left ? parkDISTANCE : -parkDISTANCE)
                .build();

        Trajectory park2Trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(park2DISTANCE)
                .build();


        waitForStart();

        if (isStopRequested()) return;
            drive.followTrajectory(leftTrajectory);
            drive.followTrajectory(forwardTrajectory);
            drive.followTrajectory(rightTrajectory);
            drive.followTrajectory(adjustTrajectory);
            elevator.runToPosition(27);
            elevator.openTrapDoor();
            Thread.sleep(2000);
            elevator.closeTrapDoor();
            elevator.runToPosition(1);
            drive.followTrajectory(parkTrajectory);
            drive.followTrajectory(park2Trajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}