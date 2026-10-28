package org.firstinspires.ftc.teamcode.teleop.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.teleop.decode.BaseTeleOpDecode;

@TeleOp(name="TuneLauncherPIDF", group="BioBuzz")

public class TuneLauncherPIDF extends BaseTeleOpDecode {
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 0;

    @Override
    protected void teleInit() {
        super.teleInit();
    }

    @Override
    protected void teleIteration() {
        PIDFCoefficients c = devices.launcher.getPIDFCoeffs();
        double increment = 0.5;

        kp = c.p;
        ki = c.i;
        kd = c.d;
        kf = c.f;

        // kp: DPAD right+/left-
        // kf: DPAD up+/down-
        // kd: y+/x-
        // ki: b+/a-

        // left 50 0 15 15
        // right 100 0 25 18

        if (launcherMorePower) {
            kd += increment;
        } else if (launcherLessPower) {
            kd -= increment;
        }

        if (slower) {
            devices.launcher.spinDown();
        } else if (faster) {
            devices.launcher.spinUp();
        }

        c = new PIDFCoefficients(kp, ki, kd, kf);
        devices.launcher.setPIDFCoeffs(c);

        telemetry.clearAll();
        telemetry.addData("PIDF current", "%.2f %.2f %.2f %.2f", kp, ki, kd, kf);
        telemetry.addData("Speed", "%.2f of %.2f",
                devices.launcher.getPower(), devices.launcher.getPowerLevel());
    }
}