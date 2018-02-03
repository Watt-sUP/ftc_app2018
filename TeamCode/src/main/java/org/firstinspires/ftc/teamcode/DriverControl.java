package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class DriverControl extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Runner runner;
    private CubeCollector collector;

    @Override
    public void runOpMode()
    {
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

        telemetry.update();

        waitForStart();
        runtime.reset();

        /// Init after start
        runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setLiftMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive())
        {
            /// Gamepad1
            runner.move(gamepad1.left_stick_x, gamepad1.right_stick_y);

            /// Gamepad2
            if(gamepad2.a)  collector.setControllingServo(2);
            if(gamepad2.y)  collector.setControllingServo(1);
            if(gamepad2.x)  collector.setControllingServo(3);

            if(gamepad2.dpad_left)  collector.openServo();
            if(gamepad2.dpad_right) collector.closeServo();

            collector.moveLift(gamepad2.left_stick_x);

            /// Telemetry
            timeTelemetry.setValue(runtime.toString());
            runner.logInformation("Power2");
            telemetry.update();
        }
    }
}
