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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Autonomous Middle", group="Linear Opmode")
@Disabled
public class AutonomousMiddle extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();
    /// Runner and Collector
    protected Runner rnr;
    protected CubeCollector collector;

    /// Sensors
    protected ModernRoboticsI2cGyro gyro;
    protected ModernRoboticsI2cRangeSensor dist_s;
    protected ColorSensor colorSensor;
    protected boolean dist_s_Target = false, dist_offset = false;

    /// Variables
    protected static int forward = 0;
    protected static int color = 0;    /// Red = 0, Blue = 1

    @Override
    public void runOpMode() {
        /// Initialize objects
        telemetry.setAutoClear(false);

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

        Telemetry.Item gyroTelemetry = telemetry.addData("Gyro", "Not initialized");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyroTelemetry.setValue("Calibrating...");
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            gyroTelemetry.setValue("Calibrating...");
            telemetry.update();
            sleep(50);
        }
        telemetry.update();

        Telemetry.Item colorTelemtry = telemetry.addData("Color", 0);
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();
        runtime.reset();

        /// TODO: get color and score jewels

        /// TODO: get down from platform and calibrate

        /// TODO: go to drawer

        /// TODO: rotate 90 degrees

        /// TODO: place cube

    }


    protected void Keep_Orientation(int Optimal_pos)
    {
        while (gyro.getHeading() != Optimal_pos)
        {
            int heading = gyro.getHeading();

            int left = 0;
            if (Optimal_pos > heading) left = heading + 360 - Optimal_pos;
            else left = heading - Optimal_pos;

            int right = 0;
            if (Optimal_pos > heading) right = Optimal_pos - heading;
            else right = Optimal_pos + 360 - heading;

            if (left < right)
                rnr.setPower(left, left, 0.1);
            else
                rnr.setPower(right, right, 0.1);
        }
    }

    // left =  1, center = 2, right = 3

        protected void go_to_drawer (int drawer_target_pos )
        {

            rnr.setPower(-0.5,0.5);

            int nr = 0;

            double last_dist = dist_s.getDistance(DistanceUnit.CM);

            while ( nr<drawer_target_pos )
                if( last_dist - dist_s.getDistance ( DistanceUnit.CM ) >= 7 )
                {
                    nr ++;
                    last_dist = dist_s.getDistance( DistanceUnit.CM );
                }

            rnr.setPower(0.0,0.0);

            //places the robot in the middle of the (night :))) drawer space
            rnr.distanceMove(10, 0.4);
        }
        protected void place_cube()
        {
            //places cube inside drawer
            rnr.distanceMove(15, 0.6);

            //drops the cube
            collector.changeState( 3 );
        }

    }
