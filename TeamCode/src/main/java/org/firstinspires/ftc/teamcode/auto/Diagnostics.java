package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.chassis.Position;

@Autonomous(name = "diagnostics", group = "robot")
public class Diagnostics extends BaseAutoDecode {
    @Override
    protected void auto(){
        Position pos;
        while(opModeIsActive()){
            devices.camera.off();
            pos = chassis.getPosition();
            telemetry.addData("x position:", pos.x);
            telemetry.addData("y position:", pos.y);
            telemetry.addData("otos heading:", pos.h);
            telemetry.update();
        }
    }
}