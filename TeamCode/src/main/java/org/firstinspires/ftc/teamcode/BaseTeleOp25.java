package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class BaseTeleOp25 extends BaseTeleOp {
    protected ScorpCannonMT leftCannon = null;
    protected ScorpCannonMT rightCannon = null;
    protected ScorpIntake intake = null;
    protected ScorpSorter sorter = null;
    protected ScorpCamera camera = null;

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

    protected UtilActionButton fireLeftActionButton = new UtilActionButton(() -> gamepad1.left_bumper);
    protected UtilActionButton fireRightActionButton = new UtilActionButton(() -> gamepad1.right_bumper);
    protected UtilActionButton intakeOnActionButton = new UtilActionButton(() -> gamepad1.dpad_left);
    protected UtilActionButton intakeReverseActionButton = new UtilActionButton(() -> gamepad1.dpad_right);
    protected UtilActionButton intakeOffActionButton = new UtilActionButton(() -> gamepad1.dpad_down);
    protected UtilActionButton intakeEmergencyActionButton = new UtilActionButton(() -> gamepad1.dpad_up);
    protected UtilActionButton leftMorePowerActionButton = new UtilActionButton(() -> gamepad1.y);
    protected UtilActionButton leftLessPowerActionButton = new UtilActionButton(() -> gamepad1.x);
    protected UtilActionButton rightMorePowerActionButton = new UtilActionButton(() -> gamepad1.b && !gamepad1.start);
    protected UtilActionButton rightLessPowerActionButton = new UtilActionButton(() -> gamepad1.a && !gamepad1.start);

    protected void teleInit() {
        super.teleInit();
        leftCannon = new ScorpCannonMotorPort(this, "left_cannon_wheel", "left_cannon_trigger", 1.0, DcMotorSimple.Direction.FORWARD);
        rightCannon = new ScorpCannonMotorPort(this, "right_cannon_wheel", "right_cannon_trigger", 0.75, DcMotorSimple.Direction.REVERSE);
        intake = new ScorpMotorIntake(this, "left_intake", "right_intake");
        sorter = new ScorpSorter(this, "sorter_servo");
        camera = new ScorpCamera(this, "camera");
        chassis.init();
    }

    protected void doRegularTasks() {
        super.doRegularTasks();
        //leftCannon.spinDownAndReleaseAfterDelay();
        //rightCannon.spinDownAndReleaseAfterDelay();
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
        leftCannon.reset();
        rightCannon.reset();
        super.tele();
    }
}