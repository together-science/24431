package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.util.Position;

@Autonomous(name = "report", group = "robot")
public class Report extends BaseAutoDecode {
    @Override
    protected void auto(){
        while (opModeIsActive()) {
            Position p = chassis.getPosition();
            telemetry.addData("hd(imu) ", "%.2f", chassis.getIMUHeading());
            telemetry.addData("hd(ppt) ", "%.2f", p.h);
            telemetry.addData("x ", "%.2f", p.x);
            telemetry.addData("y ", "%.2f", p.y);
            telemetry.update();
        }
    }
}

