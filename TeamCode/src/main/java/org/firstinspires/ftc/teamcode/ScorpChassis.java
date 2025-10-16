package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ScorpChassis implements RobotChassis {
    private final DcMotor      lf;
    private final DcMotor      rf;
    private final DcMotor      lb;
    private final DcMotor      rb;
    private final SparkFunOTOS otos;
    private final IMU          imu;
    private final LinearOpMode op;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7*(24.0/35.0);
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_CORRECTION       = 1.0;
    static final double     DRIVE_SPEED_FAST        = 0.4;
    static final double     DRIVE_SPEED_NORMAL      = 0.2;
    static final double     DRIVE_SPEED_SLOW        = 0.1;
    static final double     TURN_SPEED              = 0.5;
    static final double     P_TURN_GAIN             = 0.02;
    static final double     P_DRIVE_GAIN            = 0.03;

    ScorpChassis(LinearOpMode op, HardwareMap hm, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {
        this.lf = hm.get(DcMotor.class, lfName);
        this.rf = hm.get(DcMotor.class, rbName);
        this.lb = hm.get(DcMotor.class, lbName);
        this.rb = hm.get(DcMotor.class, rbName);
        this.otos = hm.get(SparkFunOTOS.class, otosName);
        this.imu = hm.get(IMU.class, imuName);
        this.op = op;
    }
    void init(){
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         * To Do:  EDIT these two lines to match YOUR mounting configuration. */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0); //This needs updating
        otos.setAngularScalar(1.0); // So does this
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

    //High level - Simple
    void moveTo(double x, double y, double driveSpeed, double accuracy){
        SparkFunOTOS.Pose2D pos;
        double posX, posY, dx, dy, h;
        double distance = 1000;
        while(Math.abs(distance) > accuracy && op.opModeIsActive()) {
            pos = otos.getPosition();
            posX = pos.x; posY = pos.y;
            posX*=-1; posY*=-1;
            dx = x - posX;
            dy = y - posY;
            distance = Math.sqrt(dx*dx+dy*dy);
            h = (Math.atan2(dy, dx)*(180/3.1415)-90);
            h = h < 0 ? h+360 : h;
            startDriveStraight(driveSpeed, h);
        }
        moveRobot(0, 0);
    }

    //Medium level - Intermediate
    void driveStraight(double maxDriveSpeed, double distance, double heading) {
        int lfTarget, rfTarget, lbTarget, rbTarget;
        double turnSpeed;
        if (op.opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            lfTarget = lf.getCurrentPosition() + moveCounts;
            rfTarget = rf.getCurrentPosition() + moveCounts;
            lbTarget = lb.getCurrentPosition() + moveCounts;
            rbTarget = rb.getCurrentPosition() + moveCounts;
            lf.setTargetPosition(lfTarget);
            rf.setTargetPosition(rfTarget);
            lb.setTargetPosition(lbTarget);
            rb.setTargetPosition(rbTarget);

            moveRobot(Math.abs(maxDriveSpeed), 0);

            while (op.opModeIsActive() && (lf.isBusy() && rf.isBusy())){
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0){
                    turnSpeed *= -1.0;
                }
                moveRobot(0, turnSpeed);
            }

            moveRobot(0, 0);
        }

    }
    void strafe(double maxDriveSpeed, double distance, double heading) {
        int lfTarget, rfTarget, lbTarget, rbTarget;
        double turnSpeed;
        if (op.opModeIsActive()) {

            double axial   = Math.cos(Math.PI/180*heading);  // Note: pushing stick forward gives negative value
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

            leftFrontPower *= maxDriveSpeed;
            rightFrontPower *= maxDriveSpeed;
            leftBackPower *= maxDriveSpeed;
            rightBackPower *= maxDriveSpeed;

            int moveCounts = (int)(distance * COUNTS_PER_INCH * (1+STRAFE_CORRECTION*Math.abs(lateral)));
            lfTarget = lf.getCurrentPosition() + (int)Math.signum(leftFrontPower)*moveCounts;
            rfTarget = rf.getCurrentPosition() +  (int)Math.signum(rightFrontPower)*moveCounts;
            lbTarget = lb.getCurrentPosition() +  (int)Math.signum(leftBackPower)*moveCounts;
            rbTarget = rb.getCurrentPosition() +  (int)Math.signum(rightBackPower)*moveCounts;

            lf.setTargetPosition(lfTarget);
            rf.setTargetPosition(rfTarget);
            lb.setTargetPosition(lbTarget);
            rb.setTargetPosition(rbTarget);

            double robotHeading = getHeading();

            strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);

            while (op.opModeIsActive() &&
                    (lf.isBusy() && rf.isBusy())) {

                turnSpeed = getSteeringCorrection(robotHeading, P_DRIVE_GAIN);

                if (distance < 0)turnSpeed*=-1;
                strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turnSpeed);
            }
            moveRobot(0, 0);
        }

    }
    void turnToHeading(double maxTurnSpeed, double heading) {
        double turnSpeed;
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (op.opModeIsActive()) { // && (Math.abs(headingError) > HEADING_THRESHOLD) <-- Might need to add
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
        }
        moveRobot(0, 0);
    }
    void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        double turnSpeed;
        while (op.opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
        }
        moveRobot(0, 0);
    }
    void startDriveStraight(double maxDriveSpeed, double h) {
        double turnSpeed = getSteeringCorrection(h, P_DRIVE_GAIN);
        moveRobot(Math.abs(maxDriveSpeed), turnSpeed);
    }
    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    //Low level - Advance
    double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = desiredHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    void moveRobot(double drive, double turn) {
        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        lf.setPower(leftSpeed);
        rf.setPower(rightSpeed);
        lb.setPower(leftSpeed);
        rb.setPower(rightSpeed);

    }
    void strafeRobot(double leftSpeed, double rightSpeed, double leftBackSpeed, double rightBackSpeed, double turn) {
        leftSpeed -= turn;
        rightSpeed += turn;
        leftBackSpeed -= turn;
        rightBackSpeed += turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
            leftBackSpeed /= max;
            rightBackSpeed /= max;
        }

        lf.setPower(leftSpeed);
        lf.setPower(rightSpeed);
        lb.setPower(leftBackSpeed);
        rb.setPower(rightBackSpeed);

    }

    @Override
    public void stop() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    @Override
    public void coast() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    @Override
    public void startDrive(double speed, double direction, double turnSpeed) {
    }

    @Override
    public void startStrafe(double speed, double direction, double heading, double turnSpeed) {
        double axial   = Math.cos(Math.PI/180*heading);  // Note: pushing stick forward gives negative value
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

        leftFrontPower *= speed;
        rightFrontPower *= speed;
        leftBackPower *= speed;
        rightBackPower *= speed;

        double turn = getSteeringCorrection(heading, turnSpeed/20);

        strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turn);
    }

    @Override
    public void startStrafe(double speed, double direction) {
    }

    @Override
    public void startTurn(double turnSpeed, double heading) {

    }
}
