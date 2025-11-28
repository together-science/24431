package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BackupBasicAuto", group = "robot")
public class BackupBasicAuto extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.patheticStartStraight();
        sleep(1500);
        chassis.patheticEndStraight();
    }
}