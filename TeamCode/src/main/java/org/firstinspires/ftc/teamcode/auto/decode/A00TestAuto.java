package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;

@Autonomous(name = "AA00 TestAuto", group = "robot")
public class A00TestAuto extends BaseAutoDecode {
    @Override
    protected void auto() {
        chassis.strafeTo(0, 24, ScorpChassisOtos.DRIVE_SPEED_NORMAL);
    }
}