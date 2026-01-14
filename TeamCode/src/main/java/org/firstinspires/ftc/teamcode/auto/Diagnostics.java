package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;
import org.firstinspires.ftc.teamcode.util.Position;

@Autonomous(name = "diagnostics", group = "robot")
public class Diagnostics extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, 45);
        chassis.strafeTo(-24, -24, ScorpChassisBase.DRIVE_SPEED_NORMAL);
        chassis.strafeTo(-24, 24, ScorpChassisBase.DRIVE_SPEED_NORMAL);
        chassis.strafeTo(24, 24, ScorpChassisBase.DRIVE_SPEED_NORMAL);
        chassis.strafeTo(24, -24, ScorpChassisBase.DRIVE_SPEED_NORMAL);
        chassis.strafeTo(0, 0, ScorpChassisBase.DRIVE_SPEED_NORMAL);
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, 0);
    }
}

//Diagnostics
//|
//-- BaseAutoDecode
//|
//-- BaseAuto
//   |
//   -- ScorpChassisPinpoint (This is the origin of getpos)