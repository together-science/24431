package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseTele extends LinearOpMode {
    protected ScorpChassis chassis = null;
    protected final ElapsedTime runtime = new ElapsedTime();

    double x = 0;
    double y = 0;
    double yaw = 0;
    boolean faster = false;
    boolean slower = false;

    protected ThresholdButton fasterButton = new ThresholdButton(() -> gamepad1.right_trigger, 0.1);
    protected ThresholdButton slowerButton = new ThresholdButton(() -> gamepad1.left_trigger, 0.1);
    protected double currentHeading;

    protected void teleInit() {
        chassis = new ScorpChassis(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "oscar", "imu");
        chassis.init();
    }

    // this method will be implemented by the sub class
    protected abstract void teleIteration();

    protected void doRegularTasks() {
    }

    protected void readGamepad() {
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        yaw = gamepad1.right_stick_x;
        faster = fasterButton.getStatus();
        slower = slowerButton.getStatus();
    }

    protected void tele() {
        // run until the end of the match (driver presses STOP)
        int iterations = 0;
        while (opModeIsActive()) {
            readGamepad();
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
        teleInit();

        telemetry.addData(">", "Waiting for start ...");
        telemetry.update();

        while (opModeInInit()) {
            sleep(50);
        }
        runtime.reset(); // keeping track of game time

        tele();
    }
}