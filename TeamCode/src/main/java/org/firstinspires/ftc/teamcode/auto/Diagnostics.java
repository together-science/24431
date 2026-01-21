package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;
import org.firstinspires.ftc.teamcode.util.Position;

@Autonomous(name = "diagnostics", group = "robot")
public class Diagnostics extends BaseAutoDecode {
    @Override
    protected void auto(){
        int v = 12;
        double speed = ScorpChassisBase.DRIVE_SPEED_FAST;
        //chassis.turnToHeading(speed, 45);
        chassis.strafeTo(-v, 0, speed);
        chassis.strafeTo(-v, -v, speed);
        chassis.strafeTo(-v, v, speed);
        chassis.strafeTo(v, v, speed);
        chassis.strafeTo(v, -v, speed);
        chassis.strafeTo(0, 0, speed);
        chassis.turnToHeading(speed, 0);
    }
}

//Diagnostics
//|
//-- BaseAutoDecode
//|
//-- BaseAuto
//   |
//   -- ScorpChassisPinpoint (This is the origin of getpos)