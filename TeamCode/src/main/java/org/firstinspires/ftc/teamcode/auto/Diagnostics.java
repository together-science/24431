package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.util.Position;

@Autonomous(name = "diagnostics", group = "robot")
public class Diagnostics extends BaseAutoDecode {
    @Override
    protected void auto(){
        Position pos;
        while(opModeIsActive()){
            chassis.camera.off();
            pos = chassis.getPosition();
            telemetry.addData("x position:", pos.x);
            telemetry.addData("y position:", pos.y);
            telemetry.addData("heading:", pos.h);
            telemetry.update();
        }
    }
}

//Diagnostics
//|
//-- BaseAutoDecode
//|
//-- BaseAuto
//   |
//   -- ScorpChassisPinpoint (This is the origin of getpos)