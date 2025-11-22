package org.firstinspires.ftc.teamcode.gm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto25;
import org.firstinspires.ftc.teamcode.ScorpChassis;

@Autonomous(name = "A000 Test GM", group="Robot")
public class A000testgm extends BaseAuto25 {
    @Override
    protected void auto(){
        chassis.strafeTo(24, 0, ScorpChassis.DRIVE_SPEED_NORMAL);
        //chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45);
    }
}