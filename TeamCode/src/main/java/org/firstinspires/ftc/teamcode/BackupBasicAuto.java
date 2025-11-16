package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A00 BasicBackupAuto", group = "robot")
public class BackupBasicAuto extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.moveTo(0, 24, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
}
