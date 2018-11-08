package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.BaseSystems.DriveSystem4Wheel;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
public class EducayshunOpMode extends BaseAutonomousOpMode {

    public EducayshunOpMode() {
        super("EducayshunOpMode");
    }

    @Override
    public void runOpMode()
    {

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready.", 0.25);

        ////
        waitForStart();
        ////

        driveSystem.driveToPositionInches(30, 1);
        sleep(3000);
        driveSystem.strafeLeftToPositionInches(30, 1);

        // FR BL BR FL

        stop();
    }

    private void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}