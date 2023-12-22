package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sploon {
    private DcMotor sploonMotor;
    public Sploon(HardwareMap hardwareMap) {
        this.sploonMotor = hardwareMap.dcMotor.get("sploonMotor");
    }
    public void setSploonMotor (double power){
        sploonMotor.setPower(power);
    }
}