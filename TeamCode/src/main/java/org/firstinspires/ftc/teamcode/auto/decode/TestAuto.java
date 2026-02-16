package org.firstinspires.ftc.teamcode.auto.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_TestAuto", group="Alpha") // Refering to the Greek alphabet.
public class TestAuto extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.strafeTo(12, 0, ScorpChassisBase.DRIVE_SPEED_NORMAL);
    }
}
