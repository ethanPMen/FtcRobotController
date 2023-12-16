package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AEONBlueLong extends LinearOpMode {
    public static double rightDISTANCE = 70;
    public static double forwardDISTANCE = 84;
    private DcMotor intakeMotor;
    public AEONBlueLong() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = AEONTeleOp.getIntakeMotor(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(rightDISTANCE)
                .build();

        Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(forwardDISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(rightTrajectory);
        telemetry.addLine("Done with right");
        telemetry.update();
        drive.followTrajectory(forwardTrajectory);
        telemetry.addLine("Done with forward");
        telemetry.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        intakeMotor.setPower(-1);
        Thread.sleep(5000);
        intakeMotor.setPower(0);
        while (!isStopRequested() && opModeIsActive()) ;
    }
}