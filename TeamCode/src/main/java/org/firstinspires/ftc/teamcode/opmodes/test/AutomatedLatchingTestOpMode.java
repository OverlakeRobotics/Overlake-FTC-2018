package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.arm.AutonomousArm;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;

@Autonomous(name = "Automated Latching")
public class AutomatedLatchingTestOpMode extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDriveSystem driveSystem = new MecanumDriveSystem(this);
        AutonomousArm arm = new AutonomousArm(this);

        waitForStart();

        driveSystem.mecanumDriveXY(-0.2,0);

    }
}
