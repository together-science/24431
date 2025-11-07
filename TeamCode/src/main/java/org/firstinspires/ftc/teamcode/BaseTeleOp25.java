package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class BaseTeleOp25 extends BaseTeleOp {
    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;

    protected boolean fireLeft = false;
    protected boolean fireRight = false;
    protected boolean intakeOn = false;
    protected boolean intakeReverse = false;
    protected boolean intakeOff = false;
    protected boolean leftCannonMorePower = false;
    protected boolean leftCannonLessPower = false;
    protected boolean rightCannonMorePower = false;
    protected boolean rightCannonLessPower = false;
    protected String intakeState = "off";

    protected ActionButton fireLeftActionButton = new ActionButton(() -> gamepad1.left_bumper);
    protected ActionButton fireRightActionButton = new ActionButton(() -> gamepad1.right_bumper);
    protected ActionButton intakeOnActionButton = new ActionButton(() -> gamepad1.dpad_left);
    protected ActionButton intakeReverseActionButton = new ActionButton(() -> gamepad1.dpad_right);
    protected ActionButton intakeOffActionButton = new ActionButton(() -> gamepad1.dpad_down);
    protected ActionButton leftMorePowerActionButton = new ActionButton(() -> gamepad1.y);
    protected ActionButton leftLessPowerActionButton = new ActionButton(() -> gamepad1.x);
    protected ActionButton rightMorePowerActionButton = new ActionButton(() -> gamepad1.b && !gamepad1.start);
    protected ActionButton rightLessPowerActionButton = new ActionButton(() -> gamepad1.a && !gamepad1.start);

    protected void teleInit() {
        super.teleInit();
        leftCannon = new ScorpCannon(this, "left_cannon_wheel", "left_cannon_trigger", 0.75, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannon(this, "right_cannon_wheel", "right_cannon_trigger", 0.75, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");

        chassis.init();
    }

    protected void doRegularTasks() {
        super.doRegularTasks();
        leftCannon.spinDownAfterDelay();
        rightCannon.spinDownAfterDelay();
    }

    protected void readGamepad() {
        super.readGamepad();
        fireLeft = fireLeftActionButton.getStatus();
        fireRight = fireRightActionButton.getStatus();
        intakeOn = intakeOnActionButton.getStatus();
        intakeReverse = intakeReverseActionButton.getStatus();
        intakeOff = intakeOffActionButton.getStatus();
    }
}