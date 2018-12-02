package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.distance.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;

/**
 * Created by idiot on 10/11/17.
 */

@Autonomous(name = "EducayshunBoi", group = "Bot")
public abstract class AutonomousOpMode extends LinearOpMode {

    MecanumDriveSystem driveSystem;
    DistanceSystem distanceSystem;

    public AutonomousOpMode() {
    }

    @Override
    public void runOpMode()
    {


        ////
        waitForStart();
        ////

        // FR BL BR FL

        stop();
    }

    private void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}
