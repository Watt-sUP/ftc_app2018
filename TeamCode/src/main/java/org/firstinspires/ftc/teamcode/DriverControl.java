package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="DriverControl", group="Linear Opmode")
//@Disabled
public class DriverControl extends LinearOpMode
{

    //Declare OpMode components
    private ElapsedTime runtime = new ElapsedTime();
    private Runner runner;
    private CubeCollector collector;
    private RelicGrabber grabber;

    @Override
    public void runOpMode()
    {
        /// Initialize objects
        telemetry.setAutoClear(false);

        Telemetry.Item timeTelemetry = telemetry.addData("Time", 0);

        Telemetry.Item runnerTelemetry = telemetry.addData("Runner", 0);
        runner = new Runner(
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

        grabber = new RelicGrabber(
                hardwareMap.get(Servo.class, "monster"),
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DcMotor.class, "pusher")
        );

        telemetry.update();

        waitForStart();
        runtime.reset();

        /// Initializations after start
        runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber.setPusherMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int[] last = {0, 0, 0, 0};
        boolean lb = false, rb = false, trg = false;

        while (opModeIsActive())
        {
            /// gamepad1
            double rnrY = -gamepad1.left_stick_y;
            double rnrX = gamepad1.right_stick_x;
            if(gamepad1.left_bumper) runner.move(rnrY, rnrX, 0.25);
            else if(gamepad1.right_bumper) runner.move(rnrY, rnrX, 0.5);
            else runner.move(rnrY, rnrX);

            /// gamepad2
            if(gamepad2.a && last[0] == 0)
            {
                collector.changeState(2);
                last[0] = 1;
            }
            else if(!gamepad2.a) last[0] = 0;

            if(gamepad2.y && last[1] == 0)
            {
                collector.changeState(1);
                last[1] = 1;
            }
            else if(!gamepad2.y) last[1] = 0;

            if(gamepad2.x && last[2] == 0)
            {
                collector.changeState(3);
                last[2] = 1;
            }
            else if(!gamepad2.x) last[2] = 0;

            if(gamepad2.b && last[3] == 0)
            {
                collector.changeState(3);
                last[3] = 1;
            }
            else if(!gamepad2.b) last[3] = 0;

            collector.moveLift(-gamepad2.left_stick_y * 0.75);

            if(gamepad2.left_bumper)
            {
                if(!lb)
                {
                    lb = true;
                    grabber.addValue(0, -0.05);
                }
            }
            else
                lb = false;

            if(gamepad2.right_bumper)
            {
                if(!rb)
                {
                    rb = true;
                    grabber.addValue(0, 0.05);
                }
            }
            else
                rb = false;

            if(gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5)
            {
                if(!trg)
                {
                    trg = true;
                    grabber.changeStateClaw();
                }
            }
            else
                trg = false;

            grabber.movePusher(-gamepad2.left_stick_y * 0.5);

            /// Telemetry
            timeTelemetry.setValue(runtime.toString());
            runner.logInformation("Position");
            collector.logInformation("Positions");
            telemetry.update();
        }
    }
}
