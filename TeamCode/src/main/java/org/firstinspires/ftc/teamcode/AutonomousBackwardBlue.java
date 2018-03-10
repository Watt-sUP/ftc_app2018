package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Backward", group="Linear Opmode")
//@Disabled

public class AutonomousBackwardBlue extends AutonomousMiddle
{
    static
    {
        forward = -1;
        color = 1;
    }
}
