package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class ScorpIntake {
    private CRServo left = null;
    private CRServo right = null;
    // private static final double SPEED = 1.0;
    private final LinearOpMode op;

    ScorpIntake(LinearOpMode op, String leftName, String rightName) {
        this.op = op;
        try {
            left = op.hardwareMap.get(CRServo.class, leftName);
            right = op.hardwareMap.get(CRServo.class, rightName);
            this.op.telemetry.addLine("found intake servos");
        } catch (Exception ignored) {
        }

        if (left != null && right != null) {
            // no encoder, brake on zero
//            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            // intake motors run in opposite directions
//            left.setDirection(DcMotor.Direction.FORWARD);
//            right.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    void on() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(-1.0);
        right.setPower(1.0);
    }

    void off() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(0);
        right.setPower(0);
    }

    void reverse() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(1.0);
        right.setPower(-1.0);
    }
}
