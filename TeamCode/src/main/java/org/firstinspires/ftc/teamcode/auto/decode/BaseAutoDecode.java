package org.firstinspires.ftc.teamcode.auto.decode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.devices.decode.DecodeDevices;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;

public abstract class BaseAutoDecode extends BaseAuto {

    protected DecodeDevices devices = null;

    @Override
    protected void autoInit() {
        devices = new DecodeDevices(this);
    }

    @Override
    protected void autoStart() {
        super.autoStart();
        devices.leftCannon.reset();
        devices.rightCannon.reset();
    }
}
