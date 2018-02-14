package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * RelicGrabber class is used for controlling the relic grabbing system of the robot #Mugurel
 */
public class RelicGrabber
{
    /**
     * monster = big servo
     * claw = small servo attached to the big servo
     */
    private Servo monster, claw;
    /**
     * pusher = push motor
     */
    private DcMotor pusher;

    /**
     * telemetry = telemetry item assigned to the moving system
     * verbose = true if any telemetry item is assigned to this class
     */
    private Telemetry.Item telemetry;
    private boolean verbose = false;

    /**
     * monsterP = constant values of positions for the big servo
     * clawP = constant values of positions for the small servo
     * pushP = constant values to limit the actions for push motor
     */
    private double monsterP[] = {0.0, 1.0};
    private double clawP[] = {0.0, 1.0};
    private int pushP[] = {0, (int)(1e6)};
    /// TODO: Get constant values

    /**
     *  stateMonster = 0 -> down, 1 -> up
     *  stateClaw = 0 -> open, 1 -> closed
     */
    private int stateMonster = 0, stateClaw = 0;

    /**
     * Constructor
     * @param _monster big servo
     * @param _claw small servo
     * @param _pusher push motor
     * @param _telemetry OPTIONAL -> telemetry item
     */
    RelicGrabber(Servo _monster, Servo _claw, DcMotor _pusher, Object... _telemetry)
    {
        monster = _monster; claw = _claw; pusher = _pusher;

        if(_telemetry.length > 0 && (_telemetry[0] instanceof Telemetry.Item))
        {
            telemetry = (Telemetry.Item) _telemetry[0];
            verbose = true;
            telemetry.setValue("OK!");
        }
        else
            verbose = false;

        pusher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set power of push motor
     * @param power new power to set
     */
    public void movePusher(double power)
    {
        //if(pusher.getCurrentPosition() < pushP[0])   { pusher.setPower(0); return; }
        //if(pusher.getCurrentPosition() > pushP[1])   { pusher.setPower(0); return; }
        pusher.setPower(power);
    }

    /**
     * Set run mode for push motor
     * @param rm new RunMode to set
     */
    public void setPusherMode(DcMotor.RunMode rm) { pusher.setMode(rm); }
}
