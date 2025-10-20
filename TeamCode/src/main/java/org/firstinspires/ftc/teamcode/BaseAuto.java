package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BaseAuto extends LinearOpMode {
    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpChassis chassis = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;
    protected ScorpColorTestMM color = null;

    protected void autoInit() {
    }

    protected void auto() {
    }

    @Override
    public void runOpMode() {
        leftCannon = new ScorpCannon(hardwareMap, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(hardwareMap, "right_cannon_wheel", "right_cannon_trigger");
        chassis = new ScorpChassis(this, hardwareMap, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        intake = new ScorpIntake(hardwareMap, "left_intake", "right_intake");
        sorter = new ScorpSorter(hardwareMap, "sorter_servo");
        camera = new ScorpCamera(this, hardwareMap, "camera");
        // color = new ScorpColorTestMM(hardwareMap, "sensor_color_not_real");
        // I commented out the init ^^^ because sensor_color_not_real, is not real so when it tires to find it it will run an error.
        chassis.init();

        autoInit();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }

        auto();
    }
}
