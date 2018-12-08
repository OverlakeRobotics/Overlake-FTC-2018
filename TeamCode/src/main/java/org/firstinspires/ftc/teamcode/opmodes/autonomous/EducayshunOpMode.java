package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

@Autonomous(name = "EducayshunOpMode", group = "Bot")
    public class EducayshunOpMode extends BaseAutonomousOpMode {
        private final String TAG = "EducationOpMode";


    public SoundPool mySound;
    public int beepID;

        public EducayshunOpMode() {
            super("EducayshunOpMode");
        }

        @Override
        public void runOpMode()
        {

            telem("About to initialize systems.", 0.25);
            this.initSystems();
            telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

            ////
            waitForStart();
            ////

            // 0 is red crator

            /////distanceSystem.getCloseToWall(18, 0.4);


            /*distanceSystem.getCloseToWall(7, -0.7);

            sleep(8000);
            telemetry.addLine("reached wall");
            telemetry.update();

            distanceSystem.driveAlongWallToCrater(3, 6, 1);

            sleep(3000);

            distanceSystem.driveAlongWallToDepot(3, 6, -1);


            //driveSystem.driveToPositionInches(30, 1);*/

        /*int ticksLeft = 3000;
        ExponentialRamp ramp = new ExponentialRamp(new Point(0, 0.1), new Point(ticksLeft, 1));

        int counts = 6;
        for (int i = 0; i < counts; i++) {
            telemetry.addLine("ticksLeft: " + ticksLeft);
            telemetry.addLine("rampX: " + ramp.scaleX(ticksLeft));
            telemetry.update();
            ticksLeft -= (ticksLeft / 6);
            sleep(1000);
        }*/


            //driveSystem.driveToPositionInches(15, 1);
        if (zone == 0) {
            for (int i = 0; i < 50000; i++) {
                distanceSystem.telemetry();
                colorSystem.telemetry();
                sleep(10);
            }
        } else if (zone == 1) {
            distanceSystem.getCloseToWall(12, 0.5);
        } else if (zone == 2) {
            markerSystem.reset();
            telem("just reset", 1);
            markerSystem.place();
            telem("just place", 1);
        } else if (zone == 4) {
            telem("about to FUK", 1);
            distanceSystem.driveAlongWallInches(300, 3, 6, 0.8, true);
        }
            // FR BL BR FL

            stop();
        }
}