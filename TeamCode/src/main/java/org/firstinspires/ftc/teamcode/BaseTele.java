package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BaseTele extends LinearOpMode {
    private ScorpCannon leftCannon = null;
    private ScorpCannon rightCannon = null;
    protected ScorpChassis chassis = null;
    private ScorpIntake intake = null;
    private ScorpSorter sorter = null;

    protected void teleInit(){
    }
    protected void tele(){
    }

    boolean IsKeyDown(String Key){
        if(Key.startsWith("a"))return gamepad1.a;
        if(Key.startsWith("b"))return gamepad1.b;
        if(Key.startsWith("x"))return gamepad1.x;
        if(Key.startsWith("y"))return gamepad1.y;
        if(Key.startsWith("dpad_down"))return gamepad1.dpad_down;
        if(Key.startsWith("dpad_up"))return gamepad1.dpad_up;
        if(Key.startsWith("dpad_left"))return gamepad1.dpad_left;
        if(Key.startsWith("dpad_right"))return gamepad1.dpad_right;
        return false;
    }

    @Override
    public void runOpMode() {
        chassis = new ScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new ScorpCannon(this, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(this, "right_cannon_wheel", "right_cannon_trigger");
        intake = new ScorpIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");

        chassis.init();

        teleInit();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }

        tele();
    }
}