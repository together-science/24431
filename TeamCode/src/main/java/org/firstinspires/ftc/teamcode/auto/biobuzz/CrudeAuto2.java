package org.firstinspires.ftc.teamcode.auto.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CrudeAuto-2")
public class CrudeAuto2 extends BaseAutoBiobuzz {
    @Override
    protected void auto() {
        chassis.crudeWheelPowersSet(0.4);
        sleep(600);
        chassis.crudeWheelPowersSet(0.4, -0.4, -0.4, 0.4);
        sleep(500);
        chassis.crudeWheelPowersSet(0);
    }
}
