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
    private DcMotor plus, red, green, blue;
    private VoltageSensor voltage_sensor;
    private Telemetry.Item telemetry;
    private boolean verbose = false;
    private ModernRoboticsUsbDcMotorController controller;

    RGBStrip(DcMotor _plus, DcMotor _red, DcMotor _green, DcMotor _blue, Object... _telemetry)
    {
        plus = _plus; red = _red; green = _green; blue = _blue;
        plus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        red.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        green.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blue.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = (ModernRoboticsUsbDcMotorController) plus.getController();
        if(_telemetry.length > 0 && (_telemetry[0] instanceof Telemetry.Item))
        {
            verbose = true;
            telemetry = (Telemetry.Item) _telemetry[0];
        }
        else
            verbose = false;
    }

    public void off()
    {
        plus.setPower(0.0);
        red.setPower(0.0);
        green.setPower(0.0);
        blue.setPower(0.0);
    }

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

        plus.setPower(0.01);
        red.setPower(rPower);
        green.setPower(gPower);
        blue.setPower(bPower);
    }

    public void logInformation()
    {
        telemetry.setValue( String.format("%.3f", plus.getPower()) + " " + String.format("%.3f", red.getPower()) + " " +
                String.format("%.3f",green.getPower()) + " " + String.format("%.3f", blue.getPower()) );
    }
}
