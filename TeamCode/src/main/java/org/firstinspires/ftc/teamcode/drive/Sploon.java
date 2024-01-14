package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sploon {
    private DcMotor sploonMotor;
    private Servo sploonServo;
    public Sploon(HardwareMap hardwareMap) {
        this.sploonMotor = hardwareMap.dcMotor.get("sploonMotor");
        this.sploonServo = hardwareMap.servo.get("sploonServo");
    }
    public void setSploonMotor (double power){
        sploonMotor.setPower(power);
    }
    public void setClimbPosition (){sploonMotor.setTargetPosition(sploonMotor.getCurrentPosition());}
    public void rotateServo (double position) {sploonServo.setPosition(position);}
}