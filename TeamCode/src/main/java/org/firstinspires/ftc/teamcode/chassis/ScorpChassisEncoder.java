package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ScorpChassisEncoder extends ScorpChassisBase {
    static final double     COUNTS_PER_MOTOR_REV    = 537.7*(24.0/35.0);
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_CORRECTION       = 1.0;


    public ScorpChassisEncoder(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String imuName) {
        super(op, lfName, rfName, lbName, rbName, imuName);
    }

    @Override
    public void init() {
        super.init();
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // target movement
    public void strafeTo(double x, double y, double driveSpeed, double heading) {
        int lfTarget, rfTarget, lbTarget, rbTarget;
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (!op.opModeIsActive()) {
            stop();
            return;
        }
        double distance = Math.sqrt(x*x + y*y);

        double axial   = Math.cos(Math.PI/180*heading);
        double lateral =  -Math.sin(Math.PI/180*heading);

        double leftFrontPower  = (axial + lateral);
        double rightFrontPower = (axial - lateral);
        double leftBackPower   = (axial - lateral);
        double rightBackPower  = (axial + lateral);

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower *= driveSpeed;
        rightFrontPower *= driveSpeed;
        leftBackPower *= driveSpeed;
        rightBackPower *= driveSpeed;

        int moveCounts = (int)(distance * COUNTS_PER_INCH * (1+STRAFE_CORRECTION*Math.abs(lateral)));
        lfTarget = lf.getCurrentPosition() + (int)Math.signum(leftFrontPower)*moveCounts;
        rfTarget = rf.getCurrentPosition() +  (int)Math.signum(rightFrontPower)*moveCounts;
        lbTarget = lb.getCurrentPosition() +  (int)Math.signum(leftBackPower)*moveCounts;
        rbTarget = rb.getCurrentPosition() +  (int)Math.signum(rightBackPower)*moveCounts;

        lf.setTargetPosition(lfTarget);
        rf.setTargetPosition(rfTarget);
        lb.setTargetPosition(lbTarget);
        rb.setTargetPosition(rbTarget);

        _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);

        while (op.opModeIsActive() && (lf.isBusy() && rf.isBusy())) {
            double turnSpeed = _getSteeringCorrection(heading, P_DRIVE_GAIN);
            _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turnSpeed);
        }
        stop();
    }
}

//void driveStraight(double maxDriveSpeed, double distance, double heading) {
//    throw new UnsupportedOperationException();
//        int lfTarget, rfTarget, lbTarget, rbTarget;
//        double turnSpeed;
//        if (op.opModeIsActive()) {
//            int moveCounts = (int)(distance * COUNTS_PER_INCH);
//            lfTarget = lf.getCurrentPosition() + moveCounts;
//            rfTarget = rf.getCurrentPosition() + moveCounts;
//            lbTarget = lb.getCurrentPosition() + moveCounts;
//            rbTarget = rb.getCurrentPosition() + moveCounts;
//            lf.setTargetPosition(lfTarget);
//            rf.setTargetPosition(rfTarget);
//            lb.setTargetPosition(lbTarget);
//            rb.setTargetPosition(rbTarget);
//
//            _moveRobot(Math.abs(maxDriveSpeed), 0);
//
//            while (op.opModeIsActive() && (lf.isBusy() && rf.isBusy())){
//                turnSpeed = _getSteeringCorrection(heading, P_DRIVE_GAIN);
//                if (distance < 0){
//                    turnSpeed *= -1.0;
//                }
//                _moveRobot(0, turnSpeed);
//            }
//
//            _moveRobot(0, 0);
//        }
//}

