package org.firstinspires.ftc.teamcode.auto.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "w")
public class BioShoot extends BaseAutoBiobuzz {
    @Override
    protected void auto() {
        chassis.strafeTo(0, 12, ScorpChassisBase.DRIVE_SPEED_FAST);
        sleep(5000);
    }
}