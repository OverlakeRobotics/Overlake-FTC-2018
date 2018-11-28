package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        telem("Initialized all systems. Ready." + ("" + disSys), 0.25);

        ////
        waitForStart();
        ////

        //disSys.driveAlongWallInches(60, 12, 1);
        //disSys.driveTest(60, 4, 16, 0.7);

        driveSystem.driveToPositionInches(15, 1);

        /*for (int i = 0; i < 5000; i++) {
            disSys.telemetry();
            sleep(10);
        }*/

        // FR BL BR FL

        stop();
    }

    private void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}