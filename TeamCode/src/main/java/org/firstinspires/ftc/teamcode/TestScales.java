package org.firstinspires.ftc.teamcode;

public class TestScales extends BaseAuto{
    @Override
    protected void auto(){
        while(opModeIsActive()){
            chassis.testScales();
        }
    }
}