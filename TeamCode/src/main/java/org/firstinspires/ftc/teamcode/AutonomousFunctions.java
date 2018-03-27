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
    protected RGBStrip led;

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
    protected VuforiaTrackables relicTrackables;
    protected VuforiaTrackable relicTemplate;

    /// Compass + Accelerometer
    protected double fXg = 0;
    protected double fYg = 0;
    protected double fZg = 0;

    /// Telemetry
    protected Telemetry.Item gyroTelemetry, rangeTelemetry, nrTelemetry, compassTelemetry, odsTelemetry, drw;
    protected Telemetry.Item lft, rgt, state;

    // initializes objects
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

        if(isStopRequested())   return;

        Telemetry.Item collectorTelemetry = telemetry.addData("Collector", 0);
        collector = new CubeCollector(
                hardwareMap.get(Servo.class, "upleft"),
                hardwareMap.get(Servo.class, "upright"),
                hardwareMap.get(Servo.class, "downleft"),
                hardwareMap.get(Servo.class, "downright"),
                hardwareMap.get(DcMotor.class, "lifter"),
                collectorTelemetry
        );

        if(isStopRequested())   return;

        //gyro calibration process
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

        if(isStopRequested())   return;

        //dist_f = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distf");
        dist_r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distr");
        //dist_s = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dists");
        //dist_f.setI2cAddress(I2cAddr.create8bit(0x28));
        dist_r.setI2cAddress(I2cAddr.create8bit(0x2a));
        //dist_s.setI2cAddress(I2cAddr.create8bit(0x2c));

        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        //compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        //sleep(200);
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        compassTelemetry = telemetry.addData("Compass", "init");

        //extender = hardwareMap.get(Servo.class, "extender");
        rotor = hardwareMap.get(Servo.class, "rotor");
        //extender.setPosition(1.0);

        if(isStopRequested())   return;

        Telemetry.Item colorTelemtry = telemetry.addData("Color", 0);
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colors");

        led = new RGBStrip(
                hardwareMap.get(DcMotor.class, "red"),
                hardwareMap.get(DcMotor.class, "green"),
                hardwareMap.get(DcMotor.class, "blue")
        );

        if(isStopRequested())   return;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AT4pxIn/////AAAAGSL31MQ2OkEAlE8fAGFaXLprsCGiwn5e81ZDhALBFUMK45pVGTrKvkFV9ZBC8LGo2fubVHcyc2JBpk2bsXVf/0zESirRkEkFjIItegiTFBB6wwQeeBcANTIZAFH1EjAo6QDGxlMTyhJ6JJQmrm2yBPFJFpfWDus1E34IVIyLHc8V/iCSTuegaorAlk1MXV7r2jog5G5rwsWhfD6CDx2E85s1cD20eEC4XQqx2pgcbG0CB1ohiPIYG0jtj373VY8gn9PfX2YJDBe/sLAx4IbxQxvtpgL5PAhAFcTdPfYAi6GmUzLbWQDITzao3dFlxiJjRA8fklV5KsBKJFj6viP+m3HybrTgW6kR+VOFbU2J2wD8";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        drw = telemetry.addData("Key", "init");

        if(isStopRequested())   return;

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
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        int step = 0;
        drw.setValue(vuMark.toString());
        telemetry.update();
        int drawer_number=0;
        while (step < 10) {
            if (vuMark == RelicRecoveryVuMark.LEFT) drawer_number = 1;
            if (vuMark == RelicRecoveryVuMark.CENTER) drawer_number = 2;
            if (vuMark == RelicRecoveryVuMark.RIGHT) drawer_number = 3;
            if(drawer_number!=0)
                break;
            step++;
        }
        if(drawer_number==0)
            drawer_number=2;
        return drawer_number;

    }

    // CALL THIS METHOD FOR SCORING JEWELS
    protected void scoreJewels(Servo extender)
    {
        rotor.setPosition(0.5);
        sleep(750);
        extender.setPosition(0.33);
        sleep(1000);

        colorSensor.enableLed(true);
        int clr = 0;
        if(colorSensor.red() > colorSensor.blue())  clr = 0;
        else if(colorSensor.red() == colorSensor.blue())    clr = -1;
        else    clr = 1;

        if(clr == -1)
        {
            rotor.setPosition(0.45);
            extender.setPosition(0.35);
            sleep(200);
            if(colorSensor.red() > colorSensor.blue())  clr = 0;
            else if(colorSensor.red() == colorSensor.blue())    clr = -1;
            else    clr = 1;
        }

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.update();

        sleep(400);

        if(clr == color)    rotor.setPosition(1.0);
        else if(clr != -1)    rotor.setPosition(0.0);
        //colorSensor.enableLed(false);

        sleep(1000);
        extender.setPosition(1);
        sleep(500);
        rotor.setPosition(0);
    }
    // THIS METHOD RETURNS THE PITCH RELATIVE TO THE HORIZONT LINE, FROM THE ACCELEROMETER SENSOR
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
    // CALL THIS METHOD TO GET DOWN FROM PLATFORM
    protected void getDown()
    {
        double power = 0.3;
        double heading = getPitch();
        double deg1 = 3.0;
        double okDegrees = 1;
        //rnr.setPower(-power * forward, power * forward);
        double last_dist = dist_r.getDistance(DistanceUnit.CM);
        double maxDist = 55.0;
        int step = 0;

        while( nr == 0 )
        {

            double currentHeading = getPitch();
            compassTelemetry.setValue(currentHeading);
            telemetry.update();
            double dist = dist_r.getDistance(DistanceUnit.CM);
            //sleep(200);
            double dif = heading - currentHeading;
            // This lines makes a proportional gain for the motors power relative to the robot pitch( inclanation ),
            // relative to the horizont. Because the difference between initial value and the current value is dropping to a point an then climbing,
            //we need to check and see where we are in this interval, so that we can scale the motors power like an Monotonic descending function
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
                //still looking for the drawer, just in case that robot gets stuck in this function
                //if( last_dist - dist >=7 )
                //    nr ++;
                double pw = power * dif * 0.16;
                rnr.setPower(-pw, pw, forward);
            }
            last_dist = dist;
            if(rnr.movedDistance() > maxDist)   return;
            if(!opModeIsActive())   return;
        }
    }

    /**
     * KEEPS THE ROBOT TO A DESIRED ORIENTATION
     * @PARAM OPTIMAL_POS : THE DESIRED ORIENTATION (IN DEGREES)
     */
    protected void Keep_Orientation(double Optimal_pos)
    {
        double okDegrees = 2;

        while (true)
        {
            double heading = getGyroHeading();

            gyroTelemetry.setValue(heading);
            telemetry.update();

            double left = 0;
            // checking to find the shortest way to travel to optimal pos , and determining the sense of the rotation(ccw/cw)
            if (Optimal_pos > heading) left = -heading + Optimal_pos;
            else left = -heading + 360 + Optimal_pos;

            double right = 0;
            if (Optimal_pos > heading) right = -Optimal_pos + heading + 360;
            else right = -Optimal_pos + heading;

            if( Math.min(left, right) <= okDegrees )
            {
                rnr.setPower(0.0, 0.0);
                return;
            }

            if (left < right)
                rnr.setPower(left, left, 0.006);//proportional turning
            else
                rnr.setPower(-right, -right, 0.006);//proportional turning

            if( !opModeIsActive() ) return;
        }
    }
    // THIS METHOD STOPS THE ROBOT IN FRONT OF THE FIRST DRAWER , IT RECOGNIZES THE DRAWER USING RANGE SENSOR
    protected void go_to_drawer ()
    {
        double initPower = 0.2;

        double orientation = getGyroHeading();
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
            gyroTelemetry.setValue(getGyroHeading());
            telemetry.update();
            sleep(200);
            if( !opModeIsActive() ) return;
        }

        rnr.setPower(0.0,0.0);
    }
    // CALL THIS METHOD TO GRAB CUBE
    protected void grab_cube()
    {
        collector.closeArms(1);
        sleep(100);
        collector.moveLift(0.5);
        sleep(300);
        collector.moveLift(0.0);
    }
    /**
     *THIS METHOD ESTABLISHES HOW MUCH OF A DISTANCE DOES THE ROBOT NEEDS TO GO AFTER SEEING THE FIRST DRAWER,TO PLACE ITSELF
     IN FRONT OF THE NEEDED DRAWER
     * @PARAM nr: VALUE RETURNED FROM getKeyDrawer()
     */

    protected void pick_drawer(int nr) {
        if (nr == 1)
            rnr.distanceMove(6 * forward, 0.3, this);
        if (nr == 2)
            rnr.distanceMove(25.5 * forward, 0.3, this);
        if (nr == 3)
            rnr.distanceMove(44 * forward, 0.3, this);
        if( !opModeIsActive() ) return;

    }
    // CALL THIS METHOD FOR PLACING THE CUBE AFTER ROTATING IN FRONT OF THE CORRECT DRAWER, GUIDED BY TIME
    protected void place_cube()
    {
        rnr.setPower(-0.3, 0.3);
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
        rnr.setPower(0, 0);
        sleep(1000);
        if(!opModeIsActive())   return;

        rnr.setPower(0.1, -0.1);
        if(!opModeIsActive())   return;
        sleep(1500);
        rnr.setPower(0.0, 0.0);
        if(!opModeIsActive())   return;

        if (!opModeIsActive()) return;
    }
    // CALL THIS METHOD FOR PLACING THE CUBE AFTER ROTATING IN FRONT OF THE CORRECT DRAWER, GUIDED BY ENCODERS
    protected void place_cube_encoders(boolean middle)
    {
        if(middle)
            rnr.distanceMove(20, 0.25, this);
        else
            rnr.distanceMove(30, 0.25, this);
        collector.openArms(3);
        sleep(500);
        rnr.distanceMove(-15, 0.25, this);
        sleep(500);

        if(runtime.seconds() > 28.0)    return;

        collector.closeArms(3);
        sleep(100);
        rnr.distanceMove(15, 0.25, this);
        collector.openArms(3);
        sleep(100);
        rnr.distanceMove(-15, 0.25, this);
    }

    protected void place_cube_time()
    {
        sleep(1000);
        rnr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rnr.setPower(-0.25, 0.25);
        sleep(1250);
        rnr.setPower(0.0, 0.0);
        collector.openArms(3);
        sleep(500);
        rnr.setPower(0.25, -0.25);
        sleep(750);
        rnr.setPower(0.0, 0.0);

        if(runtime.seconds() > 28.0)    return;

        collector.closeArms(3);
        sleep(100);
        rnr.setPower(-0.25, 0.25);
        sleep(1000);
        rnr.setPower(0.0, 0.0);
        collector.openArms(3);
        sleep(200);
        rnr.setPower(0.25, -0.25);
        sleep(750);
        rnr.setPower(0.0, 0.0);
    }

    // THIS METHOD GETS THE REAL  INTEGRATED Z VALUE FROM THE GYRO SENSOR ( THE ONE RETURNED BY DEFAULT IS NOT CORRECT IN OUR CASE)
    protected double gyroGetIntegratedZ()
    {
        double angle = gyro.getIntegratedZValue();
        double scale = 360.0 / 346.0;
        angle *= scale;
        return angle;
    }
    // THIS METHOD CONVERTS FROM INTEGRATED Z VALUE TO HEADING ( THE OANE RETURNED BY DEFAULT IS NOT CORRECT)
    protected double getGyroHeading()
    {
        double angle = gyroGetIntegratedZ();
        while(angle < 0)    angle += 360;
        while(angle >= 360)  angle -= 360;
        return angle;
    }
}