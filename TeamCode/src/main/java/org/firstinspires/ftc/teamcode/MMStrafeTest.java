package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="MMStrafeTest", group="Robot")

public class MMStrafeTest extends BaseAuto {

    protected void auto() {
        chassis.strafe(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,0);
        chassis.strafe(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,-90);
        chassis.strafe(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,180);
        chassis.strafe(ScorpChassis.DRIVE_SPEED_NORMAL,10.0,90);
    }
}
