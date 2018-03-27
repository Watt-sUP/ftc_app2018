package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Red Backward", group="Linear Opmode")
//@Disabled

public class AutonomousBackward extends AutonomousMiddle
{
    static
    {
        forward = -1;
        color = 0;
    }
}
