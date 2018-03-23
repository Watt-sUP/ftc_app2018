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
    private RGBStrip led;

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
                hardwareMap.get(DcMotor.class, "monster"),
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DcMotor.class, "pusher")
        );

        led = new RGBStrip(
                hardwareMap.get(DcMotor.class, "red"),
                hardwareMap.get(DcMotor.class, "green"),
                hardwareMap.get(DcMotor.class, "blue")
        );

        Servo extender, rotor;
        extender = hardwareMap.get(Servo.class, "extender");
        rotor = hardwareMap.get(Servo.class, "rotor");
        extender.setPosition(1.0);

        telemetry.update();

        waitForStart();
        runtime.reset();

        /// Initializations after start
        runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber.setPusherMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int[] last = {0, 0, 0, 0, 0};
        boolean lb = false, rb = false, trg = false;
        int forward = 1;
        int ledon = 1;
        boolean backpressed = false;
        while (opModeIsActive())
        {
            /// gamepad1 functions
         // WHEN DPAD DOWN / UP BUTTON IS PRESSED DURIN DRIVER CONTROLLED PERIOD SWITCH ROBOT FORWARD AND BACKWARD CONTROL
            if(gamepad1.dpad_down)
            {
                if(forward == 1) runner.swapMotors();
                forward = -1;
            }
            if(gamepad1.dpad_up)
            {
                if(forward == -1)   runner.swapMotors();
                forward = 1;
            }

            double rnrY = -gamepad1.left_stick_y;
            double rnrX = gamepad1.right_stick_x;
            if(gamepad1.left_trigger >= 0.5) runner.move(rnrY, rnrX, 0.25);
            else if(gamepad1.right_trigger >= 0.5) runner.move(rnrY, rnrX, 0.5);
            else runner.move(rnrY, rnrX);
            if(gamepad1.y && last[4] == 0)
            {
                collector.closeArms(3);
                last[4] = 1;
            }
            else if(!gamepad1.y)    last[4] = 0;

            /// gamepad2 functions
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

            /*if(gamepad2.left_bumper)
                grabber.powerMonster(-0.1);
            else if(gamepad2.right_bumper)
                grabber.powerMonster(0.5);
            else
                grabber.powerMonster(0.0);*/
            //grabber.powerMonster(-gamepad2.right_stick_y);

            if(gamepad2.right_bumper)
                grabber.movePusher(-0.2);
            else if(gamepad2.left_bumper)
                grabber.movePusher(0.2);
            else
                grabber.movePusher(0.0);

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
            // Reversing gamepad x axis
            double g2ry = -gamepad2.right_stick_y;
            if(g2ry < 0)
            {
                if(gamepad2.dpad_down)  grabber.powerMonster(g2ry * 0.5);
                else grabber.powerMonster(g2ry * 0.1);
            }
            else
            {
                if(gamepad2.dpad_up)    grabber.powerMonster(g2ry * 0.5);
                else    grabber.powerMonster(g2ry * 0.3);
            }
            if(gamepad2.back)
            {
                if(!backpressed)
                {
                    ledon ^= 1;
                    backpressed = true;
                }
            }
            else
                backpressed = false;
            //grabber.movePusher(g2ry * 0.25);
            /*if(g2ry == 0)   grabber.movePusher(0.0);
            else if(g2ry > 0)   grabber.movePusher(g2ry * 0.1);
            else if(g2ry < 0)   grabber.movePusher(g2ry * 0.05);*/

            /// RGB Strip
            if(ledon > 0)
            {
                if(runtime.seconds() <= 90.0)
                {
                    int sec = (int)runtime.seconds();
                    sec %= 30;
                    if(sec < 15)    led.setColor(255,153, 0);
                    else    led.setColor(0, 0, 255);
                }
                else if(runtime.seconds() < 109.0)
                    led.setColor(255, 0, 0);
                else if(runtime.seconds() >= 109.5 && runtime.seconds() <= 110.5)
                    led.setColor(0, 255, 0);
                else
                    led.setColor(255, 0, 0);
            }

            /// Telemetry
            timeTelemetry.setValue(runtime.toString());
            runner.logInformation("Position");
            collector.logInformation("Positions");
            telemetry.update();
        }
    }
}