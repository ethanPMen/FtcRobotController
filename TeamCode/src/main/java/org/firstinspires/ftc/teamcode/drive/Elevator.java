package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.AEONTeleOp;

public class Elevator {
    private DcMotor elevatorMotor;
    private Servo trapdoorServo; // Ethan Likes Men
    private Elevator(HardwareMap hardwareMap) {
        this.elevatorMotor = AEONTeleOp.getElevatorMotor(hardwareMap);
        trapdoorServo = hardwareMap.servo.get("trapdoorServo");
    }

    private static Elevator instance = null;
    public static Elevator getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Elevator(hardwareMap);
        }
        return instance;
    }

    private boolean runningToPosition;
    private double positionCmdInches;

    public void runToPosition(double positionInches) {
        positionCmdInches = positionInches;
        runningToPosition = true;
        busy = true;
        waitForIdle();
    }

    public void openTrapDoor() {
        trapdoorServo.setPosition(0);
    }

    public void closeTrapDoor() {
        trapdoorServo.setPosition(1);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    private boolean busy;
    final double kElevatorScale = 1.0 / 100.0;

    public boolean isBusy() {
        return this.busy;
    }
    private double lastError = 0;
    final double kP = 1.0 / 3.0;
    final double kD = 0;

    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    private void update() {
        if (runningToPosition) {
            // Put logic for running to position with PID
            double error = positionCmdInches  - elevatorMotor.getCurrentPosition() * kElevatorScale;
            if (telemetry != null) {
                telemetry.addData("Elevator Error Inches", error);
            }
            double changeInError = error - lastError;
            double elevatorPower = kP * error + kD * changeInError;
            elevatorMotor.setPower(elevatorPower);
            lastError = error;
            if (Math.abs(error) < 1) {
                runningToPosition = false;
                busy = false;
                elevatorMotor.setPower(0);
            }
        }
        if (telemetry != null) {
            telemetry.update();
        }
    }
}
