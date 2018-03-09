package org.firstinspires.ftc.teamcode;

import android.support.annotation.Keep;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class AutonomousMiddle extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    /// Runner and Collector
    protected Runner rnr;
    protected CubeCollector collector;

    /// Sensors
    protected ModernRoboticsI2cGyro gyro;
    protected ModernRoboticsI2cRangeSensor dist_r;
    protected ModernRoboticsI2cCompassSensor compass;
    protected ModernRoboticsI2cColorSensor colorSensor;
    protected ModernRoboticsAnalogOpticalDistanceSensor ods;

    /// Servo
    protected Servo extender, rotor;

    /// Variables
    protected static int forward = 0;
    protected static int color = 0;    /// Red = 0, Blue = 1

    /// Vuforia
    protected VuforiaLocalizer vuforia;

    /// Compass + Accelerometer
    protected double fXg = 0;
    protected double fYg = 0;
    protected double fZg = 0;

    /// Telemetry
    Telemetry.Item gyroTelemetry, rangeTelemetry, nrTelemetry, compassTelemetry, odsTelemetry;
    Telemetry.Item lft, rgt;

    @Override
    public void runOpMode() {

        /// Initialize objects
        telemetry.setAutoClear(false);

        Telemetry.Item state = telemetry.addData("State", "init");

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

        //extender = hardwareMap.get(Servo.class, "extender");
        //rotor = hardwareMap.get(Servo.class, "rotor");

        gyroTelemetry = telemetry.addData("Gyro", "Not initialized");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        ods=hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class,"ods");
        gyroTelemetry.setValue("Calibrating...");
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            gyroTelemetry.setValue("Calibrating...");
            telemetry.update();
            sleep(50);
        }
        gyroTelemetry.setValue("Calibrated!");

        dist_r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distr");

        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        //compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //sleep(200);
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        compassTelemetry = telemetry.addData("Compass", "init");

        Telemetry.Item colorTelemtry = telemetry.addData("Color", 0);
        //colorSensor = hardwareMap.get(ColorSensor.class, "color");

        odsTelemetry = telemetry.addData("ODS", "init");
        telemetry.update();
        ods.enableLed(true);
        //if( !opModeIsActive() ) return;

        waitForStart();
        runtime.reset();

        rnr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ////////////////////////// START

        /// Grab cube

        state.setValue("grab cube");
        telemetry.update();
        grab_cube();
        if (!opModeIsActive()) return;

        /// Gets key drawer
        state.setValue("get key drawer");
        telemetry.update();
        /*
        int drawer = getKeyDrawer();
        if (forward == 1) drawer = 4 - drawer;
        if (!opModeIsActive()) return;

        /// TODO: get color and score jewels
        state.setValue("score jewels");
        telemetry.update();
        //scoreJewels();
        if(!opModeIsActive())   return;

        /// Get down from platform
        state.setValue("get down from platform");
        telemetry.update();
        getDown();
        if(!opModeIsActive())   return;

*/
        /// Go in front of first drawer
        state.setValue("go to drawer");
        telemetry.update();
        go_to_drawer();
        if (!opModeIsActive()) return;

        /// TODO: Go to needed drawer
       state.setValue("go to needed drawer");
        telemetry.update();
        pick_drawer(3);
        /*
        /// Rotate 90 degrees
        state.setValue("rotate");
        telemetry.update();
        Keep_Orientation(270);
        if( !opModeIsActive() ) return;

        /// Place cube
        state.setValue("place cube");
        telemetry.update();
        //place_cube();
        if( !opModeIsActive() ) return;
        */

        /// TODO: get more cubes
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

    protected void scoreJewels()
    {
        rotor.setPosition(0.5);
        sleep(100);
        extender.setPosition(0.6);

        int clr = 0;
        if(colorSensor.red() > colorSensor.blue())  clr = 0;
        else    clr = 1;

        if(clr == color)    rotor.setPosition(0.0);
        else    rotor.setPosition(1.0);

        sleep(100);
        extender.setPosition(0.0);
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
        double lastDif = 0.0;
        double heading = getPitch();
        double deg1 = 3.0;
        double okDegrees = 1;
        //rnr.setPower(-power * forward, power * forward);

        int step = 0;

        while( true )
        {
            double currentHeading = getPitch();
            compassTelemetry.setValue(currentHeading);
            telemetry.update();

            double dif = heading - currentHeading;

            if(step == 0)
            {
                if(dif >= deg1)  step++;
                rnr.setPower(-power, power);
            }
            else
            {
                if(dif <= okDegrees)
                {
                    rnr.setPower(0.0, 0.0);
                    break;
                }

                if(lastDif < dif)
                {
                    rnr.setPower(-power, power);
                }
                else
                {
                    double pw = power * dif * 0.12;
                    rnr.setPower(-pw, pw);
                }
            }

            lastDif = dif;

            if(!opModeIsActive())   return;
        }

        Keep_Orientation(0);
    }


    protected void Keep_Orientation(int Optimal_pos)
    {
        int okDegrees = 2;

        while (gyro.getHeading() != Optimal_pos)
        {
            gyroTelemetry.setValue(gyro.getHeading());
            telemetry.update();

            int heading = gyro.getHeading();

            int left = 0;
            if (Optimal_pos > heading) left = -heading + Optimal_pos;
            else left = -heading + 360 + Optimal_pos;

            int right = 0;
            if (Optimal_pos > heading) right = -Optimal_pos + heading + 360;
            else right = -Optimal_pos + heading;

            if( Math.min(left, right) <= okDegrees ) return;

            if (left < right)
                rnr.setPower(left, left, 0.008);
            else
                rnr.setPower(-right, -right, 0.008);

            if( !opModeIsActive() ) return;
        }
    }

    protected void go_to_drawer ()
    {
        double initPower = 0.2;

        int orientation = gyro.getHeading();
        rnr.setPower(-1,1, initPower * forward);

        int nr = 0;
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

        //places the robot in the middle of the (night :))) drawer space
        //rnr.distanceMove(10 * forward, 0.4);
    }

    protected void place_cube()
    {
        //places cube inside drawer
        rnr.distanceMove(15, 0.6);

        //drops the cube
        collector.openArms(3);
    }

    protected void grab_cube()
    {
        collector.closeArms(1);
        collector.moveLift(0.0);
        sleep(500);
        collector.moveLift(0.0);
    }
    protected void pick_drawer(int nr) {
        if (nr == 1)
            rnr.distanceMove(10, 0.3);
        if (nr == 2)
            rnr.distanceMove(30, 0.4);
        if (nr == 3)
            rnr.distanceMove(50, 0.5);
        if( !opModeIsActive() ) return;

    }
   /* private void place_cube()
    {
        double pw;
        while(true)
        {
            pw=( dist_f-8 )*( 1 / 22 );
                rnr.setPower(-pw,pw);
                if( pw <= 0.05 && pw >= 0.0 )
                {
                    break;
                    rnr.setPower(0.0,0.0);
                }
        }
        collector.openArms(3);
        if (!opModeIsActive()) return;
    }*/
}
