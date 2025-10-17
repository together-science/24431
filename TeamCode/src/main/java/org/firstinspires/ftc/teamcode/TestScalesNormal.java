package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestScales Normal", group = "Sensor")
public class TestScalesNormal extends BaseAuto{
    @Override
    protected void auto(){
        chassis.moveTo(0, 24, ScorpChassis.DRIVE_SPEED_NORMAL);
        while(opModeIsActive()){
            telemetry.addData("POS", chassis.getPositionString());
            telemetry.update();
        }
    }
}