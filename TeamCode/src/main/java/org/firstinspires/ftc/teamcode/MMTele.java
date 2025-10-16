package org.firstinspires.ftc.teamcode;

public class MMTele extends BaseTele{
    double initalizeBeforeStart;
    @Override
    protected void teleInit(){
        initalizeBeforeStart = 28;
    }

    @Override
    protected void tele(){
        initalizeBeforeStart*=0.5;
        telemetry.addData("Var:", initalizeBeforeStart);
        telemetry.update();
    }
}
