package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Autonomous Jewel", group="Linear Opmode")
@Disabled
public class AutonomousJewel extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    /// Runner and Collector
    protected Runner rnr;
    protected CubeCollector collector;

    /// Sensors
    protected ModernRoboticsI2cColorSensor colorSensor;

    /// Servo
    protected Servo extender, rotor;

    /// Variables
    protected static int color = 0;    /// Red = 0, Blue = 1

    @Override
    public void runOpMode()
    {

        /// Initialize objects
        telemetry.setAutoClear(false);

        extender = hardwareMap.get(Servo.class, "extender");
        rotor = hardwareMap.get(Servo.class, "rotor");
        extender.setPosition(1.0);

        Telemetry.Item state = telemetry.addData("State", "init");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colors");
        colorSensor.enableLed(true);

        waitForStart();
        runtime.reset();

        ////////////////////////// START


        scoreJewels();


    }

    protected void scoreJewels()
    {
        rotor.setPosition(0.45);
        sleep(750);
        extender.setPosition(0.33);
        sleep(1000);

        colorSensor.enableLed(true);
        int clr = 0;
        if(colorSensor.red() > colorSensor.blue())  clr = 0;
        else    clr = 1;

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.update();

        sleep(400);

        if(clr == color)    rotor.setPosition(1.0);
        else    rotor.setPosition(0.0);
        //colorSensor.enableLed(false);

        sleep(1000);
        extender.setPosition(1);
        sleep(1000);
        rotor.setPosition(1);

    }
}
