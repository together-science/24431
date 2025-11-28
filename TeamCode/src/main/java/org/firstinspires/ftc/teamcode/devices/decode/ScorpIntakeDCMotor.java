package org.firstinspires.ftc.teamcode.devices.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ScorpIntakeDCMotor {
    private DcMotor left = null;
    private DcMotor right = null;
    private static final double SPEED = 1.0;
    private final LinearOpMode op;

    ScorpIntakeDCMotor(LinearOpMode op, String leftName, String rightName) {
        this.op = op;
        try {
            left = op.hardwareMap.get(DcMotor.class, leftName);
            right = op.hardwareMap.get(DcMotor.class, rightName);
            this.op.telemetry.addLine("found intake servos");
        } catch (Exception ignored) {
        }

        if (left != null && right != null) {
            // no encoder, brake on zero
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // intake motors run in opposite directions
            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.REVERSE);
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
