package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue01", group="Robot")
public class Blue01 extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.moveTo(0, 60, ScorpChassis.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 180); // Left 45
        leftCannon.fire();
        sleep(1000);
        rightCannon.fire();
        //chassis.moveTo(-24, 12, ScorpChassis.DRIVE_SPEED_NORMAL); (find the player loading spot cords)
    }
}