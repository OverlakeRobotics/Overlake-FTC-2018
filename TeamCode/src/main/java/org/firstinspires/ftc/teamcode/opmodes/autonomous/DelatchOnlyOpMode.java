package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "De-latch Only", group = "Competition")
public class DelatchOnlyOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        initSystems();
        waitForStart();
        delatch();
    }
}
