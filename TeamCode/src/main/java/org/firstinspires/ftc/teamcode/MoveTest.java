package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Move Test", group="Linear Opmode")
//@Disabled
public class MoveTest extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Runner runner;
    //private CubeCollector collector;

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

        telemetry.update();

        waitForStart();
        runtime.reset();

        /// Init after start
        runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive())
        {
            /// Gamepad1
            double rnrY = -gamepad1.left_stick_y;
            double rnrX = gamepad1.right_stick_x;
            if(gamepad1.left_bumper) runner.move(rnrY, rnrX, 0.25);
            else if(gamepad1.right_bumper) runner.move(rnrY, rnrX, 0.5);
            else runner.move(rnrY, rnrX);

            /// Telemetry
            timeTelemetry.setValue(runtime.toString());
            runner.logInformation("Power2");
            telemetry.update();
        }
    }
}
