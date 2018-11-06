package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
public class EducayshunOpMode extends BaseAutonomousOpMode {

    public EducayshunOpMode() {
        super("EducayshunOpMode");
    }

    @Override
    public void runOpMode()
    {

        telem("About to initialize systems.");
        this.initSystems();
        telem("Initialized all systems. Ready.");

        ////
        waitForStart();
        ////


        /*telem("About to turn 90 deg at power 1");
        driveSystem.turn(90, 1);

        telem("About to turn -90 deg at power 1");
        driveSystem.turn(-90, 1);*/

        //sleep(2000);
        //telem("About to drive to position inches 20000 ticks at power 1");
        //driveSystem.driveToPositionInches(20000, 1);

        driveSystem.driveToPositionInches(30, 1);

        telem("just drovetopositioninches 30. about to drivetoposshortt 30");

        driveSystem.driveToPoShortt(30, 1);

        // FR BL BR FL

        //telem("just drove 2000, about to find with eye for 10 sec");
        //eye.find(10);
        //telem("done finding for 10 seconds");

        stop();
    }

    private void telem(String message) {
        telemetry.addLine(message);
        telemetry.update();
        sleep(3000);
    }
}