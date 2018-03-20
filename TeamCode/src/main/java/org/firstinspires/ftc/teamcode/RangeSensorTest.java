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

        while(true)
        {
            Keep_Orientation(90);
            break;
            //if(false)   break;
            //if(!opModeIsActive())   return;
        }
        if(true)    return;

        Telemetry.Item df, dr, ds;
        df = telemetry.addData("Range fata", "init");
        dr = telemetry.addData("Range dreapta", "init");
        ds = telemetry.addData("Range spate", "init");
        telemetry.update();

        while(true)
        {
            df.setValue(dist_f.cmUltrasonic());
            dr.setValue(dist_r.cmUltrasonic());
            ds.setValue(dist_s.cmUltrasonic());
            telemetry.update();
            if(false)   break;
            if(!opModeIsActive())   return;
        }
    }

}
