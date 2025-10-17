package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestScales slow", group = "Sensor")
public class TestScalesSlow extends BaseAuto{
    @Override
    protected void auto(){
        chassis.moveTo(0, 24, ScorpChassis.DRIVE_SPEED_SLOW);
        while(opModeIsActive()){
            telemetry.addData("POS", chassis.getPositionString());
            telemetry.update();
        }
    }
}