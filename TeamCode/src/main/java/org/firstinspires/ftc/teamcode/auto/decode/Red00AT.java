package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "Red00AT", group="Robot")
public class Red00AT extends BaseAutoDecode {
    @Override
    protected void auto(){
        // spin up cannon, see if we can get obelisk id
        devices.leftCannon.spinUp();
        devices.rightCannon.spinUp();
        getObeliskId();

        // strafe forward to firing point, try to get id one more time
        chassis.strafeTo(0, 78, ScorpChassisBase.DRIVE_SPEED_FAST);
        getObeliskId();

        // turn toward goal, fire
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, -45);
        fire();

        // turn back towards obelisk, drive backward close to base, turn toward human player
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, 45);
        chassis.strafeTo(0, -53, ScorpChassisBase.DRIVE_SPEED_FAST);
        chassis.turnToHeading(ScorpChassisBase.DRIVE_SPEED_NORMAL, 100);
    }
}