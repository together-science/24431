package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "BackupBasicAuto", group = "robot")
public class BackupBasicAuto extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.startStrafe(ScorpChassisBase.DRIVE_SPEED_NORMAL, 0);
        sleep(1500);
        chassis.stop();
    }
}