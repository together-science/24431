package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BaseTele extends LinearOpMode {
    private ScorpCannon leftCannon = null;
    private ScorpCannon rightCannon = null;
    private ScorpChassis chassis = null;
    private ScorpIntake intake = null;
    private ScorpSorter sorter = null;

    protected void teleInit(){
    }
    protected void tele(){
    }

    @Override
    public void runOpMode() {
        chassis = new ScorpChassis(this, hardwareMap, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new ScorpCannon(hardwareMap, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(hardwareMap, "right_cannon_wheel", "right_cannon_trigger");
        intake = new ScorpIntake(hardwareMap, "left_intake", "right_intake");
        sorter = new ScorpSorter(hardwareMap, "sorter_servo");

        chassis.init();

        teleInit();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }

        tele();
    }
}