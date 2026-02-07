package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.decode.BaseAutoDecode;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Arrays;

@Autonomous(name = "TickCalibration", group = "robot")
public class TickCalibration extends BaseAutoDecode {
    @Override
    protected void auto(){
        calibrateTickAveragePer(1000, 0.3, 10);
        //1000 -102.5
    }

    public void calibrateTickAveragePer(int ticks, double power, int accuracy){
        Position[] positions = new Position[accuracy];
        for(int i = 0; i < accuracy; i++){
            chassis.resetPositionAndHeading();
            sleep(1000);
            chassis.turnToTicks(ticks, power);
            telemetry.addData("Test Number: ", i+1);
            telemetry.update();
            positions[i] = chassis.getPosition();
        }

        double headingAverage = 0;
        for(int i = 0; i < accuracy; i++){
            headingAverage += positions[i].h;
        }
        headingAverage /= accuracy;

        while(opModeIsActive()) {
            telemetry.addData("Average: ", headingAverage);
            telemetry.update();
        }
    }
}