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
import com.qualcomm.robotcore.hardware.I2cAddr;
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

@Autonomous(name="Autonomous Middle", group="Linear Opmode")
@Disabled
public abstract class AutonomousFunctions extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    /// Runner and Collector
    protected Runner rnr;
    protected CubeCollector collector;

    /// Sensors
    protected ModernRoboticsI2cGyro gyro;
    protected ModernRoboticsI2cRangeSensor dist_r,dist_f,dist_s;
    protected ModernRoboticsI2cCompassSensor compass;
    protected ModernRoboticsI2cColorSensor colorSensor;

    /// Servo
    protected Servo rotor;


    /// Variables
    protected static int forward = 0;
    protected static int color = 0;    /// Red = 0, Blue = 1
    protected int nr=0;
    /// Vuforia
    protected VuforiaLocalizer vuforia;

    /// Compass + Accelerometer
    protected double fXg = 0;
    protected double fYg = 0;
    protected double fZg = 0;

    /// Telemetry
    protected Telemetry.Item gyroTelemetry, rangeTelemetry, nrTelemetry, compassTelemetry, odsTelemetry;
    protected Telemetry.Item lft, rgt, state;

    protected void initialization()
    {
        telemetry.setAutoClear(false);

        state = telemetry.addData("State", "init");

        Telemetry.Item timeTelemetry = telemetry.addData("Time", 0);

        Telemetry.Item runnerTelemetry = telemetry.addData("Runner", 0);
        rnr = new Runner(
                hardwareMap.get(DcMotor.class, "frontleft"),
                hardwareMap.get(DcMotor.class, "backleft"),
                hardwareMap.get(DcMotor.class, "frontright"),
                hardwareMap.get(DcMotor.class, "backright"),
                runnerTelemetry
        );

        Telemetry.Item collectorTelemetry = telemetry.addData("Collector", 0);
        collector = new CubeCollector(
                hardwareMap.get(Servo.class, "upleft"),
                hardwareMap.get(Servo.class, "upright"),
                hardwareMap.get(Servo.class, "downleft"),
                hardwareMap.get(Servo.class, "downright"),
                hardwareMap.get(DcMotor.class, "lifter"),
                collectorTelemetry
        );

        gyroTelemetry = telemetry.addData("Gyro", "Not initialized");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyroTelemetry.setValue("Calibrating...");
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            gyroTelemetry.setValue("Calibrating...");
            telemetry.update();
            sleep(50);
        }
        gyroTelemetry.setValue("Calibrated!");

        dist_f = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distf");
        dist_r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distr");
        dist_s = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dists");
        dist_f.setI2cAddress(I2cAddr.create8bit(0x28));
        dist_r.setI2cAddress(I2cAddr.create8bit(0x2a));
        dist_s.setI2cAddress(I2cAddr.create8bit(0x2c));

        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        //compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //sleep(200);
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        compassTelemetry = telemetry.addData("Compass", "init");

        //extender = hardwareMap.get(Servo.class, "extender");
        rotor = hardwareMap.get(Servo.class, "rotor");
        //extender.setPosition(1.0);

        Telemetry.Item colorTelemtry = telemetry.addData("Color", 0);
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colors");

        rnr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Gets key drawer with Vuforia
     * LEFT = 1
     * CENTER = 2
     * RIGHT = 3
     * UNKNOWN = 2
     *
     * @return key drawer
     */
    protected int getKeyDrawer() {
        if (true)
            return 1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AT4pxIn/////AAAAGSL31MQ2OkEAlE8fAGFaXLprsCGiwn5e81ZDhALBFUMK45pVGTrKvkFV9ZBC8LGo2fubVHcyc2JBpk2bsXVf/0zESirRkEkFjIItegiTFBB6wwQeeBcANTIZAFH1EjAo6QDGxlMTyhJ6JJQmrm2yBPFJFpfWDus1E34IVIyLHc8V/iCSTuegaorAlk1MXV7r2jog5G5rwsWhfD6CDx2E85s1cD20eEC4XQqx2pgcbG0CB1ohiPIYG0jtj373VY8gn9PfX2YJDBe/sLAx4IbxQxvtpgL5PAhAFcTdPfYAi6GmUzLbWQDITzao3dFlxiJjRA8fklV5KsBKJFj6viP+m3HybrTgW6kR+VOFbU2J2wD8";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.LEFT) return 1;
        if (vuMark == RelicRecoveryVuMark.CENTER) return 2;
        if (vuMark == RelicRecoveryVuMark.RIGHT) return 3;
        return 2;
    }

    protected void scoreJewels(Servo extender)
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

    protected double getPitch()
    {
        double alpha = 0.5;
        Acceleration acc = compass.getAcceleration();

        double X = acc.xAccel;
        double Y = acc.yAccel;
        double Z = acc.zAccel;

        double Xg, Yg, Zg;
        Xg = X;
        Yg = Y;
        Zg = Z;

        fXg = Xg * alpha + (fXg * (1.0 - alpha));
        fYg = Yg * alpha + (fYg * (1.0 - alpha));
        fZg = Zg * alpha + (fZg * (1.0 - alpha));

        double pitch = ( Math.atan2(-fXg, Math.sqrt(fYg * fYg + fZg * fZg)) * 180.0 ) / Math.PI;
        return pitch;
    }

    protected void getDown()
    {
        double power = 0.3;
        double heading = getPitch();
        double deg1 = 3.0;
        double okDegrees = 1;
        //rnr.setPower(-power * forward, power * forward);
        double last_dist = dist_r.getDistance(DistanceUnit.CM);
        int step = 0;

        while( nr==0)
        {

            double currentHeading = getPitch();
            compassTelemetry.setValue(currentHeading);
            telemetry.update();
            double dist = dist_r.getDistance(DistanceUnit.CM);
            //sleep(200);
            double dif = heading - currentHeading;

            if(step == 0)
            {
                if(dif >= deg1)  step++;
                rnr.setPower(-power, power, forward);
            }
            else
            {
                if(dif <= okDegrees)
                {
                    rnr.setPower(0.0, 0.0);
                    Keep_Orientation(0);
                    break;
                }

                if( last_dist - dist >=7 )
                    nr ++;
                double pw = power * dif * 0.12;
                rnr.setPower(-pw, pw, forward);
            }
            last_dist = dist;
            if(!opModeIsActive())   return;
        }

        Keep_Orientation(0);
    }


    protected void Keep_Orientation(int Optimal_pos)
    {
        int okDegrees = 1;

        while (true)
        {
            int heading = gyro.getHeading();

            gyroTelemetry.setValue(heading);
            telemetry.update();

            int left = 0;
            if (Optimal_pos > heading) left = -heading + Optimal_pos;
            else left = -heading + 360 + Optimal_pos;

            int right = 0;
            if (Optimal_pos > heading) right = -Optimal_pos + heading + 360;
            else right = -Optimal_pos + heading;

            if( Math.min(left, right) <= okDegrees ) return;

            if (left < right)
                rnr.setPower(left, left, 0.004);
            else
                rnr.setPower(-right, -right, 0.004);

            if( !opModeIsActive() ) return;
        }
    }

    protected void go_to_drawer ()
    {
        double initPower = 0.2;

        int orientation = gyro.getHeading();
        rnr.setPower(-1,1, initPower * forward);


        double last_dist = dist_r.getDistance(DistanceUnit.CM);

        rangeTelemetry = telemetry.addData("Range", String.format("%.3f", last_dist));
        nrTelemetry = telemetry.addData("Nr", nr);
        lft = telemetry.addData("Left", -1);
        rgt = telemetry.addData("Right", -1);
        telemetry.update();

        while ( nr < 1 )
        {
            //Keep_Orientation(orientation);
            double dist = dist_r.getDistance(DistanceUnit.CM);
            if( last_dist - dist >= 5 )
                nr ++;
            last_dist = dist;
            rnr.setPower(-1,1, initPower * forward);

            rangeTelemetry.setValue( String.format("%.3f", dist));
            nrTelemetry.setValue(nr);
            gyroTelemetry.setValue(gyro.getHeading());
            telemetry.update();
            sleep(200);
            if( !opModeIsActive() ) return;
        }

        rnr.setPower(0.0,0.0);
    }

    protected void grab_cube()
    {
        collector.closeArms(1);
        //collector.moveLift(0.5);
        //sleep(300);
        collector.moveLift(0.0);
    }
    protected void pick_drawer(int nr) {
        if (nr == 1)
            rnr.distanceMove(10 * forward, 0.3);
        if (nr == 2)
            rnr.distanceMove(26 * forward, 0.3);
        if (nr == 3)
            rnr.distanceMove(41 * forward, 0.3);
        if( !opModeIsActive() ) return;

    }
    protected void place_cube()
    {
        rnr.setPower(-0.5, 0.5);
        sleep(1000);
        rnr.setPower(0, 0);
        /*dist_r = null;
        dist_f = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distf");
        double pw=0.3;
        while(dist_f.getDistance(DistanceUnit.CM)>=23)
        {
            rnr.setPower(-pw,pw);

        }
        rnr.setPower(0.0,0.0);*/

        collector.openArms(3);
        sleep(500);
        if(!opModeIsActive())   return;

        rnr.setPower(0.5, -0.5);
        if(!opModeIsActive())   return;
        sleep(500);
        rnr.setPower(0.0, 0.0);
        if(!opModeIsActive())   return;

        if (!opModeIsActive()) return;
    }
}