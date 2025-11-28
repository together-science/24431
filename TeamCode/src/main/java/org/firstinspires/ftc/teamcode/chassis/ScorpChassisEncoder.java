package org.firstinspires.ftc.teamcode.chassis;

public class ScorpChassisEncoder {
}
// This stuff is important for the encoder
//
//static final double     COUNTS_PER_MOTOR_REV    = 537.7*(24.0/35.0);
//static final double     DRIVE_GEAR_REDUCTION    = 1.0;
//static final double     WHEEL_DIAMETER_INCHES   = 4.0;
//static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
//static final double     STRAFE_CORRECTION       = 1.0;
//public static final double     DRIVE_SPEED_FAST        = 0.4;
//public static final double     DRIVE_SPEED_NORMAL      = 0.3;
//public static final double     DRIVE_SPEED_SLOW        = 0.2;
//static final double     HEADING_THRESHOLD       = 2.0;
//static final double     P_TURN_GAIN             = 0.05;
//static final double     P_DRIVE_GAIN            = 0.03;
//static final double     ACCURACY                = 2.0;
//static final boolean    DEBUG                   = true;

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

//public void strafeDistance(final double maxDriveSpeed, final double distance, final double heading) {
//    throw new UnsupportedOperationException();
//        int lfTarget, rfTarget, lbTarget, rbTarget;
//        double turnSpeed;
//        if (op.opModeIsActive()) {
//
//            double axial   = Math.cos(Math.PI/180*heading);  // Note: pushing stick forward gives negative value
//            double lateral =  -Math.sin(Math.PI/180*heading);
//
//            double leftFrontPower  = (axial + lateral);
//            double rightFrontPower = (axial - lateral);
//            double leftBackPower   = (axial - lateral);
//            double rightBackPower  = (axial + lateral);
//
//            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//            }
//
//            leftFrontPower *= maxDriveSpeed;
//            rightFrontPower *= maxDriveSpeed;
//            leftBackPower *= maxDriveSpeed;
//            rightBackPower *= maxDriveSpeed;
//
//            int moveCounts = (int)(distance * COUNTS_PER_INCH * (1+STRAFE_CORRECTION*Math.abs(lateral)));
//            lfTarget = lf.getCurrentPosition() + (int)Math.signum(leftFrontPower)*moveCounts;
//            rfTarget = rf.getCurrentPosition() +  (int)Math.signum(rightFrontPower)*moveCounts;
//            lbTarget = lb.getCurrentPosition() +  (int)Math.signum(leftBackPower)*moveCounts;
//            rbTarget = rb.getCurrentPosition() +  (int)Math.signum(rightBackPower)*moveCounts;
//
//            lf.setTargetPosition(lfTarget);
//            rf.setTargetPosition(rfTarget);
//            lb.setTargetPosition(lbTarget);
//            rb.setTargetPosition(rbTarget);
//
//            double robotHeading = getHeading();
//
//            _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);
//
//            while (op.opModeIsActive() &&
//                    (lf.isBusy() && rf.isBusy())) {
//
//                turnSpeed = _getSteeringCorrection(robotHeading, P_DRIVE_GAIN);
//
//                if (distance < 0)turnSpeed*=-1;
//                _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turnSpeed);
//            }
//            _moveRobot(0, 0);
//        }
//}