package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test", group="Robot")

public class MMStrafeTest extends BaseAuto {

    protected void auto() {
        strafe(DRIVE_SPEED_NORMAL,10.0,0);
        strafe(DRIVE_SPEED_NORMAL,10.0,-90);
        strafe(DRIVE_SPEED_NORMAL,10.0,180);
        strafe(DRIVE_SPEED_NORMAL,10.0,90);
    }
}
