package org.firstinspires.ftc.teamcode.devices.biobuzz;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannon;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannonMotorPort;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpIntakeDCMotor;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpSorter;

public class Devices {
    public Launcher launcher;

    public Devices(LinearOpMode op) {
        launcher = new Launcher(op, "launcher", "rigger_l", "trigger_r",
                 0.75, DcMotorSimple.Direction.FORWARD,
                new PIDFCoefficients(50, 0.2, 30, 15));
//        intake = new Intake(op, "intake");
//        flipper = new Flipper(op, "flipper");
        //camera = new ScorpCamera(op, "Webcam 1");
    }
}