package org.firstinspires.ftc.teamcode.individual.mm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.chassis.ScorpCamera;

public class Chassis {
    protected DcMotor lf = null;
    protected DcMotor rf = null;
    protected DcMotor lb = null;
    protected DcMotor rb = null;
    protected IMU imu = null;
    protected LinearOpMode op = null;
    public ScorpCamera camera;

    public Chassis(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String imuName) {
        try {
            this.lf = op.hardwareMap.get(DcMotor.class, lfName);
            this.rf = op.hardwareMap.get(DcMotor.class, rfName);
            this.lb = op.hardwareMap.get(DcMotor.class, lbName);
            this.rb = op.hardwareMap.get(DcMotor.class, rbName);
            this.imu = op.hardwareMap.get(IMU.class, imuName);
            this.op = op;
            this.camera = new ScorpCamera(op, "Webcam 1");
        } catch (Exception ignored) {}
    }

    void _moveRobot(double drive, double turn) {
        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        lf.setPower(leftSpeed); rf.setPower(rightSpeed);
        lb.setPower(leftSpeed); rb.setPower(rightSpeed);
    }
}
