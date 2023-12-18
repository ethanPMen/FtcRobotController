package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(group = "drive")
public class AEONBlueShort extends AEONRedShort {
    public AEONBlueShort() {
        super(false);
    }
}