package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.chassis.ScorpChassisBase;
import org.firstinspires.ftc.teamcode.chassis.ScorpChassisOdometry;
import org.firstinspires.ftc.teamcode.util.ThresholdButton;

public abstract class BaseTeleOp extends LinearOpMode {
    public ScorpChassisBase chassis = null;
    public final ElapsedTime runtime = new ElapsedTime();

    public double x = 0;
    public double y = 0;
    public double yaw = 0;
    public boolean faster = false;
    public boolean slower = false;

    public ThresholdButton fasterButton = new ThresholdButton(() -> gamepad1.right_trigger, 0.1);
    public ThresholdButton slowerButton = new ThresholdButton(() -> gamepad1.left_trigger, 0.1);
    public double currentHeading;

    protected void teleInit() {
        chassis = new ScorpChassisOdometry(this, "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", "imu");
        chassis.init();
    }

    // this method will be implemented by the sub class
    protected abstract void teleIteration();

    protected void doRegularTasks() {
    }

    protected void readGamepad() {
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;
        yaw = -gamepad1.right_stick_x;
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