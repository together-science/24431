package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BackupBasicAuto", group = "robot")
public class BackupBasicAuto extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.patheticStartStraight();
        sleep(1000);
        chassis.patheticEndStraight();
    }
}