package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.tensorflow.TensorFlow;

import java.util.List;


@Autonomous(name = "Competition Autonomous")
public class CraterOpMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String TAG = "TensorFlowTelemetry";
    private static final int DISTANCE_TO_CRATER = 36;

    private static final int SCREEN_WIDTH = 1280;
    private static final int SCREEN_CENTER = 1280 / 2;
    private static final int OFFSET = 50;

    private TensorFlow tensorFlow;
    private MecanumDriveSystem driveSystem;
    private ArmSystem armSystem;
    private System lineSystem;
    private boolean hasDriven;
    private boolean hasCheckedLeft;

    @Override
    public void runOpMode() {
        hasDriven = false;
        hasCheckedLeft = false;
        initializeOpMode();
        waitForStart();

        if (opModeIsActive()) {
            tensorFlow.activate();
            lookForGoldMineral();
        }
        tensorFlow.shutDown();
    }

    private void initializeOpMode() {
        driveSystem = new MecanumDriveSystem(this);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tensorFlow = new TensorFlow(this);
            tensorFlow.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
            tensorFlow.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    private void lookForGoldMineral() {
        while (shouldLookForGoldMineral()) {
            List<Recognition> updatedRecognitions = tensorFlow.getUpdatedRecognitions();
            if (shouldHandleUpdatedRecognitions(updatedRecognitions)) {
                handleUpdatedRecognitions(updatedRecognitions);
            }
        }
    }

    private boolean shouldLookForGoldMineral() {
        return !hasDriven;
    }

    private boolean shouldHandleUpdatedRecognitions(List<Recognition> updatedRecognitions) {
        return updatedRecognitions.size() > 0;
    }

    private void handleUpdatedRecognitions(List<Recognition> updatedRecognitions) {
        int goldMineralX = -1;

        telemetry.addData("# Object Detected", updatedRecognitions.size());

        for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = (int) recognition.getBottom();
            }
        }

        if (!hasFoundGoldMineral(goldMineralX)  ) {
            turnAndSearch();
        } else {
            handleGoldMineralWhenFound(goldMineralX);
        }
        telemetry.update();
    }

    private void turnAndSearch() {
        if (hasCheckedLeft) {
            driveSystem.turn(40,0.5);
        } else {
            driveSystem.turn(-20,0.5);
            hasCheckedLeft = true;
        }
    }

    private boolean shouldStrafeRight(int silverMineral2X) {
        return silverMineral2X < SCREEN_WIDTH * 2 / 3;
    }

    private void handleGoldMineralWhenFound(int goldMineralX) {
        if (goldMineralX < SCREEN_CENTER - OFFSET) {
            // strafe right to center gold
            Log.i(TAG, "strafing right to center gold");
            driveStrafe(0, -0.2, 250);
        } else if (goldMineralX > SCREEN_CENTER + OFFSET) {
            // strafe left to center gold
            Log.i(TAG, "strafing left to center gold");
            driveStrafe(0, 0.2, 250);
        } else {
            Log.i(TAG, "driving forward to hit gold -- gold seen");
            driveSystem.turn(-90, 0.5);
            driveToCrater();
            hasDriven = true;
        }
    }

    private void centerMineral() {

    }

    private void driveStrafe(double x, double y, int miliseconds) {
        driveSystem.mecanumDriveXY(x, y);
        sleep(miliseconds);
        driveSystem.mecanumDriveXY(0, 0);
    }

    private void driveToCrater() {
        hasDriven = true;
        driveSystem.driveToPositionInches(DISTANCE_TO_CRATER, 0.3);
    }

    private boolean hasFoundGoldMineral(int goldMineralX) {
        return  goldMineralX != -1;
    }
}
