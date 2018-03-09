package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="Servo Config", group="Linear Opmode")
//@Disabled
public class ServoConfig extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int L = 0;
        Servo[] servoArray = new Servo[25];
        servoArray[L] = hardwareMap.get(Servo.class, "upleft"); L++;
        servoArray[L] = hardwareMap.get(Servo.class, "upright"); L++;
        servoArray[L] = hardwareMap.get(Servo.class, "downleft"); L++;
        servoArray[L] = hardwareMap.get(Servo.class, "downright"); L++;
        servoArray[L] = hardwareMap.get(Servo.class, "monster"); L++;
        servoArray[L] = hardwareMap.get(Servo.class, "claw"); L++;

        for(int i = 0; i < L; i++)  servoArray[i].setPosition(0.5);

        int id = 0;

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        boolean up = false, down = false;
        boolean left = false, right = false;

        while (opModeIsActive())
        {
            if(gamepad1.dpad_right)
            {
                if(!right)
                {
                    right = true;
                    id++;
                    if(id > servoArray.length)  id = 0;
                }
            }
            else
                right = false;

            if(gamepad1.dpad_left)
            {
                if(!left)
                {
                    left = true;
                    id--;
                    if(id < 0) id = servoArray.length - 1;
                }
            }
            else
                left = false;

            Servo servo = (Servo)servoArray[id];
            double pos = servo.getPosition();

            if(gamepad1.dpad_up)
            {
                if(!up)
                {
                    up = true;
                    pos += 0.05;
                    pos = Math.min(1.0, pos);
                }
            }
            else
                up = false;

            if(gamepad1.dpad_down)
            {
                if(!down)
                {
                    down = true;
                    pos -= 0.05;
                    pos = Math.max(0.0, pos);
                }
            }
            else
                down = false;
            servo.setPosition(pos);

            telemetry.addData("Name", servo.getDeviceName());
            telemetry.addData("Manuf", servo.getManufacturer());
            telemetry.addData("Port", servo.getPortNumber());
            telemetry.addData("Controller", servo.getController().toString());
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
