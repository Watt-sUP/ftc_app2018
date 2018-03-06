package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    protected ColorSensor colorSensor;

    /// Variables
    protected static int forward = 0;
    protected static int color = 0;    /// Red = 0, Blue = 1

    /// Vuforia
    protected VuforiaLocalizer vuforia;

    /// Telemetry
    Telemetry.Item gyroTelemetry, rangeTelemetry, nrTelemetry;

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

        gyroTelemetry = telemetry.addData("Gyro", "Not initialized");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyroTelemetry.setValue("Calibrating...");
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            gyroTelemetry.setValue("Calibrating...");
            telemetry.update();
            sleep(50);
        }
        gyroTelemetry.setValue("Calibrated!");
        telemetry.update();

        dist_r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distr");

        telemetry.update();

        Telemetry.Item colorTelemtry = telemetry.addData("Color", 0);
        //colorSensor = hardwareMap.get(ColorSensor.class, "color");

        //if( !opModeIsActive() ) return;

        waitForStart();
        runtime.reset();

        rnr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ////////////////////////// START

        /// Grab cube
        state.setValue("grab cube");
        telemetry.update();
        grab_cube();
        if( !opModeIsActive() ) return;

        /// Gets key drawer
        state.setValue("get key drawer");
        telemetry.update();
        int drawer = getKeyDrawer();
        if(forward == 1)    drawer = 4 - drawer;
        if( !opModeIsActive() ) return;

        /// TODO: get color and score jewels

        /// Go in front of drawer
        state.setValue("go to drawer");
        telemetry.update();
        go_to_drawer(drawer);
        if( !opModeIsActive() ) return;

        /// Rotate 90 degrees
        state.setValue("rotate");
        telemetry.update();
        Keep_Orientation(90);
        if( !opModeIsActive() ) return;

        /// Place cube
        state.setValue("place cube");
        telemetry.update();
        place_cube();
        if( !opModeIsActive() ) return;

        /// TODO: get more cubes
    }

    /**
     * Gets key drawer with Vuforia
     * LEFT = 1
     * CENTER = 2
     * RIGHT = 3
     * UNKNOWN = 2
     * @return  key drawer
     */
    protected int getKeyDrawer()
    {
        if(true)   return 1;

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

        if(vuMark == RelicRecoveryVuMark.LEFT)  return 1;
        if(vuMark == RelicRecoveryVuMark.CENTER)    return 2;
        if(vuMark == RelicRecoveryVuMark.RIGHT) return 3;
        return 2;
    }

    /*protected void  Keep_Orientation  (int Optimal_pos ){
        while( !(Optimal_pos - 3 <= gyro.getHeading() && gyro.getHeading() <= Optimal_pos + 3) ) {
            gyroTelemetry.setValue(gyro.getHeading());
            telemetry.update();
            // test to rotate to the smaller angle
            //ratio
            //ToDo: test if  rotation constant is big enough for rotation to be made when Optimal_pos-gyro.getHeading is small, try a bigger/smaller constant
            if (Optimal_pos - gyro.getHeading() < 0)
                rnr.setPower((Optimal_pos - gyro.getHeading()) * 0.04, -(Optimal_pos - gyro.getHeading()) * 0.04);//proportional rotation
            else if (Optimal_pos - gyro.getHeading() > 0)
                rnr.setPower((Optimal_pos - gyro.getHeading()) * 0.04, -(Optimal_pos - gyro.getHeading()) * 0.04);//proportional rotation
        }
    }*/


    protected void Keep_Orientation(int Optimal_pos)
    {
        int okDegrees = 5;

        while (gyro.getHeading() != Optimal_pos)
        {
            gyroTelemetry.setValue(gyro.getHeading());
            telemetry.update();

            int heading = gyro.getHeading();

            int left = 0;
            if (Optimal_pos > heading) left = heading + 360 - Optimal_pos;
            else left = heading - Optimal_pos;

            int right = 0;
            if (Optimal_pos > heading) right = Optimal_pos - heading;
            else right = Optimal_pos + 360 - heading;

            if( Math.min(left, right) <= okDegrees ) return;

            if (left < right)
                rnr.setPower(left, left, 0.005);
            else
                rnr.setPower(-right, -right, 0.005);

            if( !opModeIsActive() ) return;
        }
    }

    protected void go_to_drawer (int drawer_target_pos )
    {
        double initPower = 0.4;

        int orientation = gyro.getHeading();
        rnr.setPower(-1,1, initPower * forward);

        int nr = 0;
        double last_dist = dist_r.getDistance(DistanceUnit.CM);

        rangeTelemetry = telemetry.addData("Range", String.format("%.3f", last_dist));
        nrTelemetry = telemetry.addData("Nr", nr);
        telemetry.update();

        while ( nr < drawer_target_pos )
        {

            double dist = dist_r.getDistance ( DistanceUnit.CM );
            if( last_dist - dist >= 7 )
                nr ++;
            last_dist = dist;
            Keep_Orientation(orientation);
            rnr.setPower(-1,1, initPower * forward);

            rangeTelemetry.setValue( String.format("%.3f", dist));
            nrTelemetry.setValue(nr);
            gyroTelemetry.setValue(gyro.getHeading());
            telemetry.update();

            if( !opModeIsActive() ) return;
        }

        rnr.setPower(0.0,0.0);

        //places the robot in the middle of the (night :))) drawer space
        rnr.distanceMove(10 * forward, 0.4);
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
}
