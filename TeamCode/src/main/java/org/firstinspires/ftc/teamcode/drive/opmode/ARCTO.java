package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Sploon;

@TeleOp
public class ARCTO extends LinearOpMode {

    double kElevatorOffset = 0;
    final double kElevatorScale = 1.0 / 100.0;

    private double getElevatorPosition() {
        return kElevatorScale * (elevatorMotor.getCurrentPosition() - kElevatorOffset);
    }

    private void reZero() {
        kElevatorOffset = elevatorMotor.getCurrentPosition();
    }

    DcMotor elevatorMotor;
    Sploon sploon;

    static DcMotor getIntakeMotor(HardwareMap hardwareMap) {
        return hardwareMap.dcMotor.get("intakeMotor");
    }

    public static DcMotor getElevatorMotor(HardwareMap hardwareMap) {
        return hardwareMap.dcMotor.get("elevatorMotor");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");
        elevatorMotor = hardwareMap.dcMotor.get("elevatorMotor");
        sploon = new Sploon(hardwareMap);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");
        Servo trapdoorServo = hardwareMap.servo.get("trapdoorServo");
        final double kP = 1.0 / 3.0;
        final double kD = 0;
        final double kI = 0;
        boolean intakeOn = false;
        boolean trapdoorBoom = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;
        double setpoint = 0;
        double lastError = 0;
        boolean prevRightBumper = gamepad1.right_bumper;
        boolean prevLeftBumper = gamepad1.left_bumper;
        boolean shouldHoldClimber = false;
        final double holdingPower = 0.5;
        while (opModeIsActive()) {
            //resetEncoders();

            double y = -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y; // Remember, Y stic/k value is reversed
            double x = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;
            boolean buttonY = gamepad1.y;
            boolean buttonB = gamepad1.b;
            boolean buttonA = gamepad1.a;
            boolean buttonX = gamepad1.x;
            // for intake toggle
            boolean rightBumperPressed = !prevRightBumper && gamepad1.right_bumper;
            prevRightBumper = gamepad1.right_bumper;
            // trapdoor toggle
            boolean leftBumperPressed = !prevLeftBumper && gamepad1.left_bumper;
            prevLeftBumper = gamepad1.left_bumper;

            //operator buttons
            boolean opY = gamepad2.y;
            boolean opB = gamepad2.b;
            boolean opA = gamepad2.a;
            boolean opX = gamepad2.x;
            boolean opdup = gamepad2.dpad_up;
            boolean opddown = gamepad2.dpad_down;
            boolean opdleft = gamepad2.dpad_left;
            boolean opdright = gamepad2.dpad_right;
            boolean opLB = gamepad2.left_bumper;
            boolean opRB = gamepad2.right_bumper;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            //intake command
            if (rightBumperPressed) {
                if (intakeOn) {
                    intakeOn = false;
                } else {
                    intakeOn = true;
                }
            }
            if (intakeOn) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            if (opRB) {
                intakeMotor.setPower(-1);
            }

            //trapdoor
            if (leftBumperPressed) {
                if (trapdoorBoom) {
                    trapdoorBoom = false;
                } else {
                    trapdoorBoom = true;
                }
                if (trapdoorBoom) {
                    trapdoorServo.setPosition(0);
                } else {
                    trapdoorServo.setPosition(1);
                }
            }

                //elevator code
                setpoint = elevatorMotor.getTargetPosition() * kElevatorScale;
                double elevatorPosition = getElevatorPosition();

                double error = setpoint - elevatorPosition;
                double changeInError = error - lastError;
                double elevatorPower = kP * error + kD * changeInError;
                lastError = error;

                // TRAPDOOR AUTO CLOSE
                if (elevatorPosition < 14.0) {
                    trapdoorServo.setPosition(1);
                }

                //elevator
                if (opA) {
                    reZero(); //sets whatever position it's at to zero
                }
                if (buttonA && (elevatorPosition > 0)) { //stow
                    elevatorMotor.setTargetPosition(0);
                }
                if (buttonX) {
                    elevatorMotor.setTargetPosition((int) (16.0 / kElevatorScale)); // low
                }
                if (buttonB) {
                    elevatorMotor.setTargetPosition((int) (27.0 / kElevatorScale)); // mid
                }
                if (buttonY && elevatorPosition < 3900 * kElevatorScale) { //high
                    elevatorMotor.setTargetPosition((int) (39.0 / kElevatorScale));
                }
                elevatorMotor.setPower(elevatorPower);

                //manual movement
                if (opY) {
                    elevatorMotor.setPower(-.5);
                    elevatorMotor.setTargetPosition(elevatorMotor.getCurrentPosition());//down
                }


                //CLIMB SPLOON
                if (opdup) {
                    sploon.setSploonMotor(1);
                } else if (opddown) {
                    sploon.setSploonMotor(-1);
                    shouldHoldClimber = false;
                } else if (shouldHoldClimber) {
                    sploon.setSploonMotor(holdingPower);
                } else {
                    sploon.setSploonMotor(0);
                }
                //SET POSITION FOR CLIMB
                if (opB) {
                    shouldHoldClimber = true;
                }
                if (opdleft) {
                    sploon.rotateServo(.1);
                } else if (opdright) {
                    sploon.rotateServo(.6);
                }

                //drone code
                if (opLB) {
                    droneServo.setDirection(Servo.Direction.REVERSE);
                    droneServo.setPosition(0); //to release
                }
                //end of drone code

                if (elevatorPosition < 14.0) {
                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);
                } else {
                    leftFront.setPower(frontLeftPower / 5);
                    leftRear.setPower(backLeftPower / 5);
                    rightFront.setPower(frontRightPower / 5);
                    rightRear.setPower(backRightPower / 5);
                }

                telemetry.addData("Elevator:", setpoint);
                telemetry.addData("Elevator Setpoint", setpoint);
                telemetry.addData("Elevator Position", getElevatorPosition());
                telemetry.addData("Elevator Target Position", elevatorMotor.getTargetPosition());
                telemetry.addData("Elevator Raw Position", elevatorMotor.getCurrentPosition());
                telemetry.addData("Error", error);

                telemetry.addData("\nDrone Servo position", droneServo.getPosition());

                telemetry.addData("\ntrapdoor position", trapdoorServo.getPosition());
                telemetry.addData("trapdoor pressed", trapdoorBoom);
                telemetry.update();
            }
        }
    }
