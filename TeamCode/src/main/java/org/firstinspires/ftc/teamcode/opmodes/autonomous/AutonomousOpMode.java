package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.distance.DistanceSystem;

/**
 * Created by idiot on 10/11/17.
 */

@Autonomous(name = "EducayshunOpMode", group = "Bot")
public abstract class AutonomousOpMode extends LinearOpMode {

    MecanumDriveSystem driveSystem;
    DistanceSystem distanceSystem;

    public AutonomousOpMode() {
    }

    @Override
    public void runOpMode()
    {

        telem("About to initialize systems.", 0.25);
        driveSystem = new MecanumDriveSystem(this);
        distanceSystem = new DistanceSystem(this, driveSystem);
        telem("Initialized all systems. Ready.", 0.25);

        ////
        waitForStart();
        ////

        for (int i = 0; i < 10000000; i++) {
            distanceSystem.findpower2(12, 1);
            sleep(5);
        }

        // FR BL BR FL

        stop();
    }

    private void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}
