package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "diagnostics", group = "robot")
public class diagnostics extends BaseAuto25 {
    @Override
    protected void auto(){
        SparkFunOTOS.Pose2D pos;
        while(opModeIsActive()){
            camera.off();
            pos = chassis.getPosition();
            telemetry.addData("x position:", pos.x);
            telemetry.addData("y position:", pos.y);
            telemetry.addData("otos heading:", pos.h);
            telemetry.update();
        }
    }
}