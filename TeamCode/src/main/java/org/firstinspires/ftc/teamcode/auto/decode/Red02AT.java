package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "_Red02", group="Robot")
public class Red02AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        // spin up cannon, see if we can get obelisk id
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        getObeliskId();

        // strafe forward to firing point, try to get id one more time
        chassis.strafeTo(0, 120, ScorpChassisBase.DRIVE_SPEED_FAST);
        getObeliskId();
        chassis.strafeTo(12, 0, ScorpChassisBase.DRIVE_SPEED_NORMAL);

        // turn toward goal, fire
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, -60);
        fire();

        // turn back towards obelisk, drive backward close to base, turn toward human player
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_SLOW, 60);
    }
}