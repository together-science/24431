package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red00", group = "robot")
public class Red00 extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.moveTo(0, 60, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45);
        leftCannon.fire();
        sleep(1000);
        rightCannon.fire();
        chassis.moveTo(12, 12, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
}
