package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Forward", group="Linear Opmode")
//@Disabled

public class AutonomousForward extends AutonomousMiddle
{
    static
    {
        forward = 1;
        color = 0;
    }
}
