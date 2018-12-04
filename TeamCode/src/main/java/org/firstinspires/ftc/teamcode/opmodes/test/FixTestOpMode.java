package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Fix")
public class FixTestOpMode extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor winch = hardwareMap.dcMotor.get("winch");
        winch.setPower(-0.2);
    }
}
