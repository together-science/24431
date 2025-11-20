package org.firstinspires.ftc.teamcode.gm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto25;
import org.firstinspires.ftc.teamcode.ScorpChassis;

@Autonomous(name = "A000 Test GM", group="Robot")
public class A000testgm extends BaseAuto25 {
    @Override
    protected void auto(){
        leftCannon.spinUp();
        rightCannon.spinUp();
        chassis.strafeTo(0, 40, ScorpChassis.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45);
        leftCannon.fire();
        sleep(1000);
        rightCannon.fire();
        sleep(1000);
        chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 0);
        chassis.strafeTo(0, 0, ScorpChassis.DRIVE_SPEED_FAST);
    }
}