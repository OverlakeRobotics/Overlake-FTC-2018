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


            
        if (zone == 0) {
            for (int i = 0; i < 50000; i++) {
                distanceSystem.telemetry();
                colorSystem.telemetry();
                sleep(10);
            }
        } else if (zone == 1) {
            distanceSystem.getCloseToWall(12, 0.5);
        } else if (zone == 2) {
            telem("about to reset",1);
            markerSystem.reset();
            telem("just reset, about to place", 1);
            markerSystem.place();
            telem("just placed", 1);
        } else if (zone == 4) {
            telem("driveAlongWallInches(300, 3, 6, 0.8, true)", 1);
            distanceSystem.driveAlongWallInches(300, 3, 6, 0.8, true);
        } else if (zone == 5) {
            craterSide();
        } else if (zone == 6) {
            distanceSystem.strafeTowardWall(1, cratApproachDeg1, 1);
        } else if (zone == 7) {
            distanceSystem.alignWithWall(1);
        } else if (zone == 8) {
            for (int i = 0; i < 50000; i++) {
                colorSystem.telemetry();
                sleep(10);
            }
        }


            // FR BL BR FL

            stop();
        }

        private void craterSide() {
            driveSystem.driveToPositionInches(backCubeIn, -autonoPower, false);
            driveSystem.turnAbsolute(cratApproachDeg0, autonoPower);
            distanceSystem.getCloseToWall(cratTargDist1, toWallPow);
            //driveSystem.turnAbsolute(cratApproachDeg1, 1);
            //distanceSystem.getCloseToWall(7, 0.8);
            driveSystem.turnAbsolute(cratApproachDeg1, autonoPower);
            parkInDepot(-autonoPower, colorSystem);
            markerSystem.reset();
            sleep(300);
            parkOnCrator(autonoPower, initPitch, initRoll);
            markerSystem.reset();
        }
}