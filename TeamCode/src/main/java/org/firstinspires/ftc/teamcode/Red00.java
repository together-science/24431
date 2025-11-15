package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red00", group = "robot")
public class Red00 extends temp00 {
    @Override
    protected void auto(){
        chassis.strafeTo(-12, 48, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 315); // right 45
        rightCannon.fire();
        sleep(2000);
        leftCannon.fire();
        chassis.moveTo(12, 12, ScorpChassis.DRIVE_SPEED_NORMAL);
    }
}
