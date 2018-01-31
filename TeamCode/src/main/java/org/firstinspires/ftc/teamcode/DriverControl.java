package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class DriverControl extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Runner runner;

    @Override
    public void runOpMode()
    {
        telemetry.setAutoClear(false);

        Telemetry.Item runnerTelemetry = telemetry.addData("Runner", 0);

        // TODO: init runner and collector
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


        while (opModeIsActive())
        {
            ;
        }
    }
}
