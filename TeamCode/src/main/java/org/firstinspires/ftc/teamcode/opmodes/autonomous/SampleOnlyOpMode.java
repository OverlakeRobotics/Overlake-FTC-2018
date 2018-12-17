package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseLinearOpMode;

@Autonomous(name = "Sample Only", group = "Competition")
public class SampleOnlyOpMode extends BaseAutonomousOpMode
{
    @Override
    public void run()
    {
        initSystems();
        waitForStart();
        sample();
    }
}
