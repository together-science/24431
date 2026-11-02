package org.firstinspires.ftc.teamcode.auto.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.biobuzz.BaseAutoBiobuzz;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "BioBasicAuto")
public class BioBasicAuto extends BaseAutoBiobuzz {
    @Override
    protected void auto() {
        chassis.strafeTo(0, 24, ScorpChassisBase.DRIVE_SPEED_SLOW);
    }
}