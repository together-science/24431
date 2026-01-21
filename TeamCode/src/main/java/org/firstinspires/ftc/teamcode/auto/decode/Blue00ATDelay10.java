package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Blue00Delay10", group="Robot")
public class Blue00ATDelay10 extends Blue00AT {
    @Override
    protected void auto() {
        sleep(10000);
        super.auto();
    }
}