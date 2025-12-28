package org.firstinspires.ftc.teamcode.auto.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;

@Autonomous(name = "PinpointAutoTestA", group = "robot")
public class PinpointAutoTestA extends BaseAutoDecode {
    @Override
    protected void auto(){
        chassis.strafeTo(0, 12, ScorpChassisBase.DRIVE_SPEED_SLOW);
    }
}
