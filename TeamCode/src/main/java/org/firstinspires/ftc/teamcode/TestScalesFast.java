package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestScales fast", group = "Sensor")
public class TestScalesFast extends BaseAuto{
    @Override
    protected void auto(){
        chassis.moveTo(0, 24, ScorpChassis.DRIVE_SPEED_FAST);
        while(opModeIsActive()){
            telemetry.addData("POS", chassis.getPositionString());
            telemetry.update();
        }
    }
}