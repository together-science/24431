package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScorpColorTestMM{
    private ColorSensor color = null;
    int r;
    int g;
    int b;

    ScorpColorTestMM(HardwareMap hm, String cameraName){
        color = hm.get(ColorSensor.class, cameraName);
    }

    void getColor(){
        r = color.red();
        g = color.green();
        b = color.blue();
    }

    int getMainColor(){
        getColor();

        if(r > g && r > b){
            return 1; // This means red
        }
        if(g > r && g > b){
            return 2; // This means green
        }
        if(b > r && b > g){
            return 3; // This means blue
        }
        return 0; // This is an error and I'm scared. If you get this, run...
    }







    
    double getPurpleXPos(){
        // Find a purple ball x, doesn't matter which one
        return 2; // Subsitute
    }
    double getPurpleYPos(){
        // Find a purple ball's y using the x getPurpleXPos()
        return 3;
    }
    double getGreenXPos(){
        // Find a green ball x, doesn't matter which one
        return -3; // Subsitute
    }
    double getGreenYPos(){
        // Find a green ball's y using the x getGreenXPos()
        return 5;
    }
}
