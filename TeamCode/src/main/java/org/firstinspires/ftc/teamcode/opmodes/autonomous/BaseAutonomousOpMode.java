package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.configs.ConfigParser;
import org.firstinspires.ftc.teamcode.opmodes.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.systems.arm.ArmDirection;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.lidar.LidarNavigationSystem;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;
import org.firstinspires.ftc.teamcode.systems.marker.Marker;
import org.firstinspires.ftc.teamcode.systems.tensorflow.TensorFlow;

import java.util.List;

/**
 * Created by EvanCoulson on 10/11/17.
 */

public abstract class BaseAutonomousOpMode extends BaseLinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String TAG = "TensorFlowTelemetry";

    private static final int CENTER = 750;
    private static final int OFFSET = 50;

    private TensorFlow tensorFlow;
    private boolean hasDriven;
    private boolean hasTurned;
    private boolean doneSearching;

    ConfigParser config;
    public MecanumDriveSystem driveSystem;
    public IMUSystem imuSystem;
    public ColorSystem colorSystem;
    public LidarNavigationSystem distanceSystem;
    public Marker markerSystem;
    public ArmSystem arm;

    public double initPitch;
    public double initRoll;

    public int zone;
    public int backCubeIn; // the inches to back up after knocking the cube
    public int cratApproachDeg0; //the angle at which the robot approaches the wall after tensor flow
    public int cratApproachDeg1; // reletive to starting position
    public int approachDeg2;
    public double cratTargDist1;
    public double powerToWall;
    public double autonomousPower;

    public double CRITICAL_ANGLE = 0.75;
    int RED_TRGGER_VALUE = 12;
    int BLUE_TRIGGER_VALUE = 8;

    public BaseAutonomousOpMode(String opModeName)
    {
        //config = new ConfigParser(opModeName + ".omc");
        config = new ConfigParser("Autonomous.omc");
        telemetry.setMsTransmissionInterval(200);
    }

    protected void initSystems()
    {
        initializeTensorFlow();
        hasDriven = false;
        hasTurned = false;
        doneSearching = false;
        this.driveSystem = new MecanumDriveSystem(this);
        this.imuSystem = new IMUSystem(this);
        colorSystem = new ColorSystem(this);
        distanceSystem = new LidarNavigationSystem(this, driveSystem, colorSystem);
        markerSystem = new Marker(this);
        arm = new ArmSystem(this);

        zone = config.getInt("zone");
        backCubeIn = config.getInt("backCubeIn");//10

        cratApproachDeg0 = config.getInt("cratApproachDeg0");//-90
        cratApproachDeg1 = config.getInt("cratApproachDeg1");//-120
        cratTargDist1 = config.getDouble("cratTargDist1");


        powerToWall = config.getDouble("ToWallPow");
        autonomousPower = config.getDouble("autonopower");

        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();
    }

    public void parkInDepot(double maxPower, ColorSystem colorSystem) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while ((colorSystem.getRed() < RED_TRGGER_VALUE) &&
                (colorSystem.getBlue() < BLUE_TRIGGER_VALUE)) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void parkOnCrator(double maxPower, double initPitch, double initRoll) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while (((Math.abs(imuSystem.getPitch() - initPitch) < CRITICAL_ANGLE) ||
                (Math.abs(imuSystem.getRoll() - initRoll) < CRITICAL_ANGLE))) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }

    public void delatch() {
        arm.toggleRamping();
        arm.runMotors(ArmDirection.DOWN);
        arm.releaseArmPin();
        sleep(1000);
        arm.stop();
        sleep(1000);
        driveSystem.mecanumDriveXY(0.3, 0);
        sleep(750);
        driveSystem.mecanumDriveXY(0, -0.3);
        sleep(500);
        driveSystem.mecanumDriveXY(-0.35,0.1);
        arm.toggleRamping();
        arm.runMotors(ArmDirection.DOWN);
        sleep(500);
        arm.stop();
        driveSystem.mecanumDriveXY(0,0);
    }

    public void sample() {
        driveSystem.mecanumDriveXY(-0.3, 0);
        sleep(700);
        driveSystem.mecanumDriveXY(0,0);
        driveSystem.driveToPositionInches(5, 0.7);
        tensorFlow.activate();
        lookForGoldMineral();
        driveSystem.driveToPositionInches(18, -1);
        if (doneSearching) {
            driveSystem.turn(125, 1);
        } else if (hasTurned) {
            driveSystem.turn(60, 1);
        } else {
            driveSystem.turn(95, 1);
        }
        tensorFlow.shutDown();
    }

    private void initializeTensorFlow() {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tensorFlow = new TensorFlow(this);
            tensorFlow.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
            tensorFlow.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

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
        int goldMineralX = getGoldMineralX(updatedRecognitions);
        if (!hasFoundGoldMineral(goldMineralX)) {
            turnAndSearch();
        } else if (hasFoundGoldMineral(goldMineralX)) {
            driveToGoldMineral();
        }
    }

    private int getGoldMineralX(List<Recognition> recognitions) {
        int goldMineralX = -1;
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = (int) recognition.getBottom();
            }
        }
        return goldMineralX;
    }

    private void turnAndSearch() {
        if (!hasTurned) {
            hasTurned = true;
            driveSystem.turn(35, 1);
        } else if (!doneSearching) {
            driveSystem.turn(-75, 1);
            doneSearching = true;
            driveToGoldMineral();
        }
    }

    private void driveToGoldMineral() {
        driveSystem.turn(-90, 1);
        driveSystem.driveToPositionInches(29, 1);
        hasDriven = true;
    }

    private boolean hasFoundGoldMineral(int goldMineralX) {
        return  goldMineralX != -1;
    }
}
