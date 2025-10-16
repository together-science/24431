package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScorpIntake {
    private DcMotor left = null;
    private DcMotor right = null;
    private static final double SPEED = 1.0;

    ScorpIntake(HardwareMap hm, String leftName, String rightName) {
        try {
            left = hm.get(DcMotor.class, leftName);
            right = hm.get(DcMotor.class, rightName);
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

    void on() {
        if (left == null || right == null) {
            return ;
        }

        left.setPower(SPEED);
        right.setPower(SPEED);
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

        left.setPower(-SPEED);
        right.setPower(-SPEED);
    }
}
