package org.firstinspires.ftc.teamcode.mm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto;
import org.firstinspires.ftc.teamcode.ScorpChassis;

@Autonomous(name="MMStrafeTest", group="Robot")

public class MMStrafeTest extends BaseAuto {

    protected void auto() {
        chassis.strafeDistance(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,0);
        chassis.strafeDistance(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,-90);
        chassis.strafeDistance(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,180);
        chassis.strafeDistance(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,90);
    }
}
