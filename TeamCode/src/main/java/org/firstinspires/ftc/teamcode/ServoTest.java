package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ServoTest", group="Linear Opmode")
@Disabled
public class ServoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CubeCollector c;

    @Override
    public void runOpMode() {


        Servo servo = hardwareMap.get(Servo.class, "upleft");
        ServoController ctrl = hardwareMap.get(ServoController.class, "Servo Controller 1");
        servo.setPosition(0.5);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //if(ctrl.isServoPwmEnabled(1))   ctrl.setServoPwmDisable(1);
            //servo.getController().pwmDisable();
            telemetry.update();
        }
    }
}
