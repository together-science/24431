package org.firstinspires.ftc.teamcode;

public abstract class BaseTeleOp25 extends BaseTeleOp {
    protected ScorpCannon leftCannon = null;
    protected ScorpCannon rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;

    boolean fireLeft = false;
    boolean fireRight = false;
    boolean intakeOn = false;
    boolean intakeReverse = false;
    boolean intakeOff = false;
    protected String intakeState = "off";

    protected ActionButton fireLeftActionButton = new ActionButton(() -> gamepad1.left_bumper);
    protected ActionButton fireRightActionButton = new ActionButton(() -> gamepad1.right_bumper);
    protected ActionButton intakeOnActionButton = new ActionButton(() -> gamepad1.dpad_left);
    protected ActionButton intakeReverseActionButton = new ActionButton(() -> gamepad1.dpad_right);
    protected ActionButton intakeOffActionButton = new ActionButton(() -> gamepad1.dpad_down);

    protected void teleInit() {
        super.teleInit();
        leftCannon = new ScorpCannon(this, "left_cannon_wheel", "left_cannon_trigger");
        rightCannon = new ScorpCannon(this, "right_cannon_wheel", "right_cannon_trigger");
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