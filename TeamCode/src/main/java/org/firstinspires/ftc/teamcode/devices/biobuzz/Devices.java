package org.firstinspires.ftc.teamcode.devices.biobuzz;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class Devices {
    public Launcher launcher;
    public Intake intake;
    public Flipper flipper;

    public Devices(BaseTeleOp op) {
        launcher = new Launcher(op, "launcher", "trigger_l", "trigger_r",
                 0.675, DcMotorSimple.Direction.FORWARD,
                new PIDFCoefficients(50, 0.2, 30, 15));
        intake = new Intake(op, "intake");
        flipper = new Flipper(op, "flipper");
        //camera = new ScorpCamera(op, "Webcam 1");
    }

    public void reset() {
        intake.off();
        flipper.close();
        launcher.reset();
    }
}