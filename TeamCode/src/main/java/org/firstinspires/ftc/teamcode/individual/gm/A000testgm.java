package org.firstinspires.ftc.teamcode.individual.gm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;

@Autonomous(name = "A000 Test GM", group="Robot")
public class A000testgm extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.strafeTo(24, 0, ScorpChassisOtos.DRIVE_SPEED_NORMAL);
        //chassis.turnToHeading(ScorpChassis.DRIVE_SPEED_NORMAL, 45);
    }
}