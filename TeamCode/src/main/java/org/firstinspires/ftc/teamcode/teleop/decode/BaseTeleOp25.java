package org.firstinspires.ftc.teamcode.teleop.decode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.devices.decode.DecodeDevices;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCamera;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpCannonMotorPort;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpMotorIntake;
import org.firstinspires.ftc.teamcode.devices.decode.ScorpSorter;
import org.firstinspires.ftc.teamcode.util.UtilActionButton;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public abstract class BaseTeleOp25 extends BaseTeleOp {
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
        super.tele();
    }
}