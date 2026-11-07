package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ScorpionLinearOpMode;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisPinpoint;
import org.firstinspires.ftc.teamcode.util.TempMessages;

public abstract class BaseAuto extends ScorpionLinearOpMode {
    protected ScorpChassisPinpoint chassis = null;
    protected void autoInit(){
        chassis = new ScorpChassisPinpoint(this,
                "lf",
                "rf",
                "lb",
                "rb",
                "don",
                "imu");
        chassis.init();
    }
    protected void autoStart() {
    }
    protected abstract void auto();

    @Override
    public void runOpMode() {
        autoInit();

        telemetry.addData(">", "Waiting for start ...");
        telemetry.update();

        while (opModeInInit()) sleep(50);
        try {
            autoStart();
            auto();
        } catch (Exception ex){
            while(opModeIsActive()){
                telemetry.clear();
                telemetry.addLine(ex.getLocalizedMessage());
                telemetry.update();
            }
        }
    }
}

// New device git agreeable test