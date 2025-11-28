package org.firstinspires.ftc.teamcode.devices.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class ScorpIntakeServo {
    private CRServo left = null;
    private CRServo right = null;
    private static final double SPEED = 1.0;
    private final LinearOpMode op;

    ScorpIntakeServo(LinearOpMode op, String leftName, String rightName) {
        this.op = op;
        try {
            left = op.hardwareMap.get(CRServo.class, leftName);
            right = op.hardwareMap.get(CRServo.class, rightName);
            this.op.telemetry.addLine("found intake servos");
        } catch (Exception ignored) {
        }
    }

    public void on() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(-SPEED);
        right.setPower(SPEED);
    }

    public void off() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(0);
        right.setPower(0);
    }

    public void reverse() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(SPEED);
        right.setPower(-SPEED);
    }
}
