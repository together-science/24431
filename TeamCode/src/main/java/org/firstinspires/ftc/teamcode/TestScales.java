package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GMTestAuto-2", group = "Sensor")
public class TestScales extends BaseAuto{
    @Override
    protected void auto(){
        while(opModeIsActive()){
            chassis.testScales();
        }
    }
}