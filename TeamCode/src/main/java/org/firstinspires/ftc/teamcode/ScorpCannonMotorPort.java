package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ScorpCannonMotorPort extends ScorpCannon {
    private DcMotor wheel = null;

    ScorpCannonMotorPort(LinearOpMode op, String wheelName, String triggerName, double power, DcMotorSimple.Direction direction) {
        super(op, triggerName, power, direction);
        try {
            this.wheel = op.hardwareMap.get(DcMotor.class, wheelName);
        } catch(Exception ignored) {
        }

        if (wheel != null) {
            wheel.setDirection(direction);
        }
    }

    protected void setPower(double power) {
        wheel.setPower(power);
    }

    protected boolean noWheel() {
        return wheel == null;
    }

    protected double getPower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return 0.0;
        }
        return wheel.getPower();
    }

}
