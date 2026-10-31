package org.firstinspires.ftc.teamcode.teleop.biobuzz;

import org.firstinspires.ftc.teamcode.devices.biobuzz.Devices;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.util.ActionButton;

public abstract class BaseTeleOpBiobuzz extends BaseTeleOp {
    protected Devices devices = null;

    protected boolean spinup = false;
    protected boolean fire = false;
    protected boolean toggleFlipper = false;
    protected boolean openFlipper = false;
    protected boolean closeFlipper = false;
    protected boolean intakeOn = false;
    protected boolean intakeReverse = false;
    protected boolean intakeOff = false;
    protected boolean launcherMorePower = false;
    protected boolean launcherLessPower = false;

    protected ActionButton spinupActionButton = new ActionButton(() -> gamepad1.left_bumper);
    protected ActionButton fireActionButton = new ActionButton(() -> gamepad1.right_bumper);
    protected ActionButton toggleActionButton = new ActionButton(() -> gamepad1.a && !gamepad1.options);
    protected ActionButton openFlipperActionButton = new ActionButton(() -> gamepad1.x && !gamepad1.options);
    protected ActionButton closeFlipperActionButton = new ActionButton(() -> gamepad1.y && !gamepad1.options);
    protected ActionButton intakeOnActionButton = new ActionButton(() -> gamepad1.dpad_left);
    protected ActionButton intakeReverseActionButton = new ActionButton(() -> gamepad1.dpad_right);
    protected ActionButton intakeOffActionButton = new ActionButton(() -> gamepad1.dpad_down);
    protected ActionButton launcherMorePowerActionButton = new ActionButton(() -> gamepad1.y);
    protected ActionButton launcherLessPowerActionButton = new ActionButton(() -> gamepad1.x);

    protected void teleInit() {
        super.teleInit();
        chassis.init();
        try {
            devices = new Devices(this);
        } catch (Exception e) {
            this.telemetry.addData("Exception:", e.getLocalizedMessage());
            this.telemetry.update();
            this.sleep(5000);
        }
    }

    protected void doRegularTasks() {
        super.doRegularTasks();
    }

    protected void readGamepad() {
        super.readGamepad();
        spinup = spinupActionButton.getStatus();
        fire = fireActionButton.getStatus();
        toggleFlipper = toggleActionButton.getStatus();
        intakeOn = intakeOnActionButton.getStatus();
        intakeReverse = intakeReverseActionButton.getStatus();
        intakeOff = intakeOffActionButton.getStatus();
        launcherMorePower = launcherMorePowerActionButton.getStatus();
        launcherLessPower = launcherLessPowerActionButton.getStatus();
    }

    protected void tele() {
        devices.reset();
        //chassis.cameraInit();
        super.tele();
    }
}