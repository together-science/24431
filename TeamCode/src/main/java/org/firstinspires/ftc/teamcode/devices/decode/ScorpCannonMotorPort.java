package org.firstinspires.ftc.teamcode.devices.decode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ScorpCannonMotorPort extends ScorpCannon {
    private DcMotorEx wheel = null;

    ScorpCannonMotorPort(LinearOpMode op, String wheelName, String triggerName, double power, DcMotorSimple.Direction direction) {
        super(op, triggerName, power, direction);
        try {
            this.wheel = op.hardwareMap.get(DcMotorEx.class, wheelName);
            this.wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.wheel.setVelocity(0);
        } catch(Exception ignored) {
        }

        if (wheel != null) {
            wheel.setDirection(direction);
        }
    }

    protected void setPower(double power) {
        wheel.setVelocity(power*2000);
    }

    protected boolean noWheel() {
        return wheel == null;
    }

    public double getPower() {
        // check if we in fact have a cannon
        if (this.noWheel()) {
            return 0.0f;
        }
        return wheel.getVelocity()/2000;
    }

}
