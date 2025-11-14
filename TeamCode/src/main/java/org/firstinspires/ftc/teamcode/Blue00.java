package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue00", group="Robot")
public class Blue00 extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.moveTo(12, 48, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45); // Left 45
        rightCannon.fire();
        sleep(2000);
        chassis.moveTo(-24, 12, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
}