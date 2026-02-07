package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisPinpoint;

public abstract class BaseAuto extends LinearOpMode {
    protected ScorpChassisPinpoint chassis = null;
    protected void autoInit(){
        chassis = new ScorpChassisPinpoint(this,
                "left_front_drive",
                "right_front_drive",
                "left_back_drive",
                "right_back_drive",
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
