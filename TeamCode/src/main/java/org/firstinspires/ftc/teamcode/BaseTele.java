package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseTele extends LinearOpMode {
    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpChassis chassis = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;
    protected final ElapsedTime runtime = new ElapsedTime();

    protected ActionButton fireLeftActionButton = new ActionButton(()->gamepad1.left_bumper);
    protected ActionButton fireRightActionButton = new ActionButton(()->gamepad1.right_bumper);
    protected ActionButton intakeOnActionButton = new ActionButton(()->gamepad1.dpad_left);
    protected ActionButton intakeReverseActionButton = new ActionButton(()->gamepad1.dpad_right);
    protected ActionButton intakeOffActionButton = new ActionButton(()->gamepad1.dpad_down);
    protected double currentHeading;
    protected String intakeState = "off";

    // this method will be implemented by the sub class
    protected void teleInit() {
        // not abstract because not required in subclass
    }
    // this method will be implemented by the sub class
    protected abstract void teleIteration();

//    boolean IsKeyDown(String Key){
//        if(Key.startsWith("a"))return gamepad1.a;
//        if(Key.startsWith("b"))return gamepad1.b;
//        if(Key.startsWith("x"))return gamepad1.x;
//        if(Key.startsWith("y"))return gamepad1.y;
//        if(Key.startsWith("dpad_down"))return gamepad1.dpad_down;
//        if(Key.startsWith("dpad_up"))return gamepad1.dpad_up;
//        if(Key.startsWith("dpad_left"))return gamepad1.dpad_left;
//        if(Key.startsWith("dpad_right"))return gamepad1.dpad_right;
//        return false;
//    }

    protected void doRegularTasks() {
        leftCannon.spinDownAfterDelay();
        rightCannon.spinDownAfterDelay();
    }


    protected void tele() {
        // run until the end of the match (driver presses STOP)
        int iterations = 0;
        while (opModeIsActive()) {
            teleIteration();
            doRegularTasks();

            // add the elapsed game time
            telemetry.addData("Base", "Run Time:  " + runtime);
            telemetry.addData("Base", "Iterations: " + (++iterations));
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        chassis = new ScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        leftCannon = new ScorpCannon(this, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(this, "right_cannon_wheel", "right_cannon_trigger");
        intake = new ScorpIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");

        chassis.init();

        teleInit();

        while (opModeInInit()) {
            telemetry.addData(">", "Waiting ...");
            telemetry.update();
        }
        runtime.reset(); // keeping track of game time

        tele();
    }
}