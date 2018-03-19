package org.firstinspires.ftc.teamcode;


import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * RGBStrip class is used to control the RGB strip attached to robot. :) (#Duta)
 */
public class RGBStrip
{
    // Initialization of all objects

    private DcMotor  red, green, blue;
    private Telemetry.Item telemetry;
    private boolean verbose = false;
    private ModernRoboticsUsbDcMotorController controller;

    /**
     * Constructor
     * @param _red :  represents the Motor controller port where the RGB Strip cathode corresponding to green is connected.
     * @param _blue :  represents the Motor controller port where the RGB Strip cathode corresponding to blue is connected.
     * @param _green :  represents the Motor controller port where the RGB Strip cathode corresponding to green is connected.
     */
    RGBStrip(DcMotor _plus, DcMotor _red, DcMotor _green, DcMotor _blue, Object... _telemetry)
    {
        red = _red; green = _green; blue = _blue;
        red.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        green.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blue.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = (ModernRoboticsUsbDcMotorController) red.getController();
        if(_telemetry.length > 0 && (_telemetry[0] instanceof Telemetry.Item))
        {
            verbose = true;
            telemetry = (Telemetry.Item) _telemetry[0];
        }
        else
            verbose = false;
    }

    //Turns off the RGB Strip


    public void off()
    {
        red.setPower(-0.01);
        green.setPower(-0.01);
        blue.setPower(-0.01);
    }

     //Set those parameters to the RGB values from the colors you need.

    public void setColor(int r, int g, int b)
    {
        if( (r == 0 && g == 0 && b == 0) || controller.getVoltage() < 11.0 )
        {
            off();
            return;
        }

        double rPower = (double)r / (double)255;
        double gPower = (double)g / (double)255;
        double bPower = (double)b / (double)255;

        if(r == 0)  rPower = -0.01;
        if(g == 0)  gPower = -0.01;
        if(b == 0)  bPower = -0.01;

        red.setPower(rPower);
        green.setPower(gPower);
        blue.setPower(bPower);
    }

    // Displays Telemetry

    public void logInformation()
    {
        telemetry.setValue( String.format("%.3f", red.getPower()) + " " +
                String.format("%.3f",green.getPower()) + " " + String.format("%.3f", blue.getPower()) );
    }
}
