package org.firstinspires.ftc.teamcode.chassis;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ScorpChassisOtos extends ScorpChassisBase {
    private SparkFunOTOS otos;

    public ScorpChassisOtos(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {
        super(op, lfName, rfName, lbName, rbName, imuName);
        try {this.otos = op.hardwareMap.get(SparkFunOTOS.class, otosName);}
        catch (Exception ignored) {}
    }
    public void init(){
        super.init();
        if (otos != null) {
            otos.setLinearUnit(DistanceUnit.INCH);
            otos.setAngularUnit(AngleUnit.DEGREES);
            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
            otos.setOffset(offset);
            otos.setLinearScalar(1.0);
            otos.setAngularScalar(1.0); // So does this
            otos.calibrateImu();
            otos.resetTracking();
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
            otos.setPosition(currentPosition);
            SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
            SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
            otos.getVersionInfo(hwVersion, fwVersion);
        }
    }
    public Position getPosition() {
        if (otos == null) {
            return null;
        }
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        Position p = new Position();
        p.x = -pos.x*1.35;
        p.y = -pos.y*0.85;
        p.h = pos.h;
        return p;
    }
    public double getHeading() {
        return getPosition().h;
    }

    public String getPositionString() {
        return "h";
    }
}