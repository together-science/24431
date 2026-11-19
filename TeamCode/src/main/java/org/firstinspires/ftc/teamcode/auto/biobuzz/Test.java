package org.firstinspires.ftc.teamcode.auto.biobuzz;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestA")
public class Test extends LinearOpMode {
    DcMotor lfdrv = null;
    DcMotor rfdrv = null;
    @Override
    public void runOpMode() {
        lfdrv = hardwareMap.get(DcMotor.class, "lf");
        rfdrv = hardwareMap.get(DcMotor.class, "rf");

        lfdrv.setPower(0.5);
        rfdrv.setPower(0.5);

        sleep(500);

        lfdrv.setPower(0);
        rfdrv.setPower(0);
    }
}
