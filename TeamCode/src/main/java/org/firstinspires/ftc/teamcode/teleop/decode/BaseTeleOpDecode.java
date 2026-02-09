package org.firstinspires.ftc.teamcode.teleop.decode;
import org.firstinspires.ftc.teamcode.devices.decode.DecodeDevices;
import org.firstinspires.ftc.teamcode.util.ActionButton;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public abstract class BaseTeleOpDecode extends BaseTeleOp {
    protected DecodeDevices devices = null;

    protected boolean fireLeft = false;
    protected boolean fireRight = false;
    protected boolean intakeOn = false;
    protected boolean intakeReverse = false;
    protected boolean intakeOff = false;
    protected boolean intakeEmergency = false;
    protected boolean leftCannonMorePower = false;
    protected boolean leftCannonLessPower = false;
    protected boolean rightCannonMorePower = false;
    protected boolean rightCannonLessPower = false;

    protected ActionButton fireLeftActionButton = new ActionButton(() -> gamepad1.left_bumper);
    protected ActionButton fireRightActionButton = new ActionButton(() -> gamepad1.right_bumper);
    protected ActionButton intakeOnActionButton = new ActionButton(() -> gamepad1.dpad_left);
    protected ActionButton intakeReverseActionButton = new ActionButton(() -> gamepad1.dpad_right);
    protected ActionButton intakeOffActionButton = new ActionButton(() -> gamepad1.dpad_down);
    protected ActionButton intakeEmergencyActionButton = new ActionButton(() -> gamepad1.dpad_up);
    protected ActionButton leftMorePowerActionButton = new ActionButton(() -> gamepad1.y);
    protected ActionButton leftLessPowerActionButton = new ActionButton(() -> gamepad1.x);
    protected ActionButton rightMorePowerActionButton = new ActionButton(() -> gamepad1.b && !gamepad1.start);
    protected ActionButton rightLessPowerActionButton = new ActionButton(() -> gamepad1.a && !gamepad1.start);

    protected void teleInit() {
        super.teleInit();
        chassis.init();
        devices = new DecodeDevices(this);
    }

    protected void doRegularTasks() {
        super.doRegularTasks();
    }

    protected void readGamepad() {
        super.readGamepad();
        fireLeft = fireLeftActionButton.getStatus();
        fireRight = fireRightActionButton.getStatus();
        intakeOn = intakeOnActionButton.getStatus();
        intakeReverse = intakeReverseActionButton.getStatus();
        intakeOff = intakeOffActionButton.getStatus();
        intakeEmergency = intakeEmergencyActionButton.getStatus();
        leftCannonMorePower = leftMorePowerActionButton.getStatus();
        leftCannonLessPower = leftLessPowerActionButton.getStatus();
        rightCannonMorePower = rightMorePowerActionButton.getStatus();
        rightCannonLessPower = rightLessPowerActionButton.getStatus();
    }

    protected void tele() {
        devices.leftCannon.reset();
        devices.rightCannon.reset();
        chassis.cameraInit();
        super.tele();
    }
}