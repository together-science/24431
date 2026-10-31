package org.firstinspires.ftc.teamcode.devices.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class Intake {
    private CRServo intakeServo = null;
    private static final double SPEED = 1.0;
    private final BaseTeleOp op;

    Intake(BaseTeleOp op, String leftName) {
        this.op = op;
        try {
            intakeServo = op.hardwareMap.get(CRServo.class, leftName);
//            op.tms.add("found intake servo");
        } catch (Exception ignored) {
        }
    }

    public void on() {
        op.tms.add("switching intake on:before check");
        if (intakeServo == null) {
            return ;
        }
        op.tms.add("switching intake on");
        intakeServo.setPower(-SPEED);
    }

    public void off() {
        if (intakeServo == null) {
            return ;
        }

        op.tms.add("switching intake off");
        intakeServo.setPower(0);
    }

    public void reverse() {
        if (intakeServo == null) {
            return ;
        }

        op.tms.add("switching intake to reverse");
        intakeServo.setPower(SPEED);
    }
}
