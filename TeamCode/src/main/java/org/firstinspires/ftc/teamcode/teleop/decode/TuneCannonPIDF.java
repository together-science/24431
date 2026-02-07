package org.firstinspires.ftc.teamcode.teleop.decode;
import static org.firstinspires.ftc.teamcode.util.Position.headingFromRelativePosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="TuneCannonPIDF", group="Linear Opmode")

public class TuneCannonPIDF extends BaseTeleOpDecode {
    String selectedCannonName = "left";

    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 0;

    @Override
    protected void teleIteration() {
        PIDFCoefficients c;
        if (selectedCannonName.equals("left")) {
            c = devices.leftCannon.getPIDFCoeffs();
        } else {
            c = devices.rightCannon.getPIDFCoeffs();
        }

        kp = c.p;
        ki = c.i;
        kd = c.d;
        kf = c.f;

        if (intakeReverse) {
            kp += 0.05;
        } else if (intakeOn) {
            kp -= 0.05;
        }

        if (intakeEmergency) {
            kf += 0.05;
        } else if (intakeOff) {
            kf -= 0.05;
        }

        if (leftCannonMorePower) {
            kd += 0.05;
        } else if (leftCannonLessPower) {
            kd -= 0.05;
        }

        if (rightCannonMorePower) {
            ki += 0.05;
        } else if (rightCannonLessPower) {
            ki -= 0.05;
        }

        // select
        if (fireLeft) {
            selectedCannonName = "left";
            devices.leftCannon.spinUp();
            devices.rightCannon.spinDown();
        }
        if (fireRight) {
            selectedCannonName = "right";
            devices.rightCannon.spinUp();
            devices.leftCannon.spinDown();
        }

        telemetry.clearAll();
        telemetry.addLine("Cannon "+selectedCannonName);
        telemetry.addData("PIDF current", "%.2f %.2f %.2f %.2f");
        if (selectedCannonName.equals("left")) {
            telemetry.addData("Left cannon", "%.2f of %.2f",
                    devices.leftCannon.getPower(), devices.leftCannon.getPowerLevel());
        } else if (selectedCannonName.equals("right")) {
            telemetry.addData("Right cannon", "%.2f of %.2f", devices.rightCannon.getPower(),
                    devices.rightCannon.getPowerLevel());
        }
    }
}