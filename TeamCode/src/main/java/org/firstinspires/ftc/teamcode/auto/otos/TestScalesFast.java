package org.firstinspires.ftc.teamcode.auto.otos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOtos;

@Autonomous(name = "TestScales fast", group = "Sensor")
public class TestScalesFast extends BaseAuto {
    @Override
    protected void auto(){
        while(opModeIsActive()){
            telemetry.addData("POS", chassis.getPositionString());
            telemetry.update();
        }
    }
}