package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by bogda on 19.03.2018.
 */

@Autonomous(name="Range Test", group="Linear Opmode")
public class RangeSensorTest extends AutonomousFunctions {

    @Override

    public void runOpMode()
    {
        Servo extender;
        extender=hardwareMap.get(Servo.class,"extender");
        extender.setPosition(1.0);

        initialization ();


        waitForStart();
        runtime.reset();

        /*while(true)
        {
            Keep_Orientation(90);
            break;
            //if(false)   break;
            //if(!opModeIsActive())   return;
        }
        if(true)    return;*/

        Telemetry.Item df, dr, ds;
        df = telemetry.addData("Red", "init");
        dr = telemetry.addData("Blue", "init");
        ds = telemetry.addData("Green", "init");
        telemetry.update();

        colorSensor.enableLed(true);
        while(true)
        {
            df.setValue(colorSensor.red());
            dr.setValue(colorSensor.blue());
            ds.setValue(colorSensor.green());
            telemetry.update();
            if(false)   break;
            if(!opModeIsActive())   return;
        }
    }

}
