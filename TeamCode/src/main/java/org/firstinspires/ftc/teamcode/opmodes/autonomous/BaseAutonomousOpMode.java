package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.systems.slide.SlideSystem;
import org.firstinspires.ftc.teamcode.systems.tensorflow.TensorFlow;

import java.sql.Time;
import java.util.Date;
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

    public TensorFlow tensorFlow;
    private boolean hasDriven;
    private boolean hasTurned;
    protected boolean doneSearching;
    private Date time;

    ConfigParser config;
    public MecanumDriveSystem driveSystem;
    public IMUSystem imuSystem;
    public ColorSystem colorSystem;
    public LidarNavigationSystem distanceSystem;
    public Marker markerSystem;
    public ArmSystem arm;
    public SlideSystem slideSystem;

    public double initPitch;
    public double initRoll;

    public int backCubeIn; // the inches to back up after knocking the cube
    public int cratApproachDeg0; //the angle at which the robot approaches the wall after tensor flow
    public int cratApproachDeg1; // reletive to starting position
    public int approachDeg2;
    public double cratToWallHeading;
    public double cratTargDist1;
    public int cratTargDist2;
    public int cratTargDist3;
    public int cratTargDist4;

    public int depDepappraochIn; // depot(OpMode) depot approach inches
    public double depWallHeading;
    public int depToCratIn;
    public double depApproachDeg0; // -90
    public double depApproachDeg1; // 45
    public double depToWallHeading; // 160

    public double toWallPow;
    public double autonoPower;
    public int inFromWall;
    public int zone;

    public double CRITICAL_ANGLE = 1.5;
    double RED_TRGGER_VALUE = 1.5;
    double BLUE_TRIGGER_VALUE = 1.5;

    public BaseAutonomousOpMode()
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
        this.slideSystem = new SlideSystem(this);
        this.imuSystem = new IMUSystem(this);
        colorSystem = new ColorSystem(this);
        distanceSystem = new LidarNavigationSystem(this, driveSystem, colorSystem);
        markerSystem = new Marker(this);
        arm = new ArmSystem(this);

        backCubeIn = config.getInt("backCubeIn");//10

        cratApproachDeg0 = config.getInt("cratApproachDeg0");//-90
        cratApproachDeg1 = config.getInt("cratApproachDeg1");//-120
        cratToWallHeading = config.getDouble("cratToWallHeading");
        cratTargDist1 = config.getDouble("cratTargDist1");
        cratTargDist2 = config.getInt("cratTargDist2");
        cratTargDist3 = config.getInt("cratTargDist3");
        cratTargDist4 = config.getInt("cratTargDist4");

        depDepappraochIn = config.getInt("depDepApproachIn");
        depWallHeading = config.getDouble("depWallHeading");
        depToCratIn = config.getInt("depToCratIn");
        depApproachDeg0 = config.getDouble("depApproachDeg0");
        depApproachDeg1 = config.getDouble("depApproachDeg1");
        depToWallHeading = config.getDouble("depToWallHeading");

        toWallPow = config.getDouble("ToWallPow");
        autonoPower = config.getDouble("autonopower");
        inFromWall = config.getInt("inFromWall");
        zone = config.getInt("zone");

        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();
    }

    public void parkInDepot(double maxPower, ColorSystem colorSystem) {
        double initBR = colorSystem.getBlue() / colorSystem.getGreen();
        double initRR = colorSystem.getRed() / colorSystem.getGreen();
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while ((colorSystem.getRed() / colorSystem.getGreen() > RED_TRGGER_VALUE) &&
                (colorSystem.getBlue() / colorSystem.getGreen() > BLUE_TRIGGER_VALUE)) {
            driveSystem.setPower(maxPower);
        }
        getTelemetry().addData("blah", "colorSystem B: " + colorSystem.getBlue());
        getTelemetry().addData("blah", "colorSystem B: " + colorSystem.getBlue());
        getTelemetry().update();
        driveSystem.setPower(0);
    }

    public void parkOnCrator(double maxPower, double initPitch, double initRoll) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.setPower(maxPower);

        while (((Math.abs(imuSystem.getPitch() - initPitch) < CRITICAL_ANGLE) ||
                (Math.abs(imuSystem.getRoll() - initRoll) < CRITICAL_ANGLE))) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void telem(String message, double sec) {
        getTelemetry().addLine(message);
        getTelemetry().update();
        sleep((int)(1000 * sec));
    }

    public void delatch() {
        arm.toggleRamping();
        arm.runMotors(ArmDirection.DOWN);
        arm.releaseArmPin();
        sleep(600);
        arm.stop();
        driveSystem.mecanumDriveXY(0.7, 0);
        sleep(500);
        driveSystem.mecanumDriveXY(0, -0.3);
        sleep(500);
        driveSystem.mecanumDriveXY(-0.42,0.1);
        collapse();
        sleep(500);
        arm.stop();
        driveSystem.mecanumDriveXY(0,0);
        driveSystem.turnAbsolute(0, 1);
    }

    public void collapse() {
        while (!arm.isCollapsed()) {
            arm.runMotors(ArmDirection.DOWN);
        }
        arm.stop();
    }

    public void sample() {
        driveSystem.driveToPositionInches(2, 1);
        driveSystem.mecanumDriveXY(-0.3, 0);
        sleep(700);
        driveSystem.mecanumDriveXY(0,0);
        tensorFlow.activate();
        lookForGoldMineral();
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
        time = new Date();
        while (shouldLookForGoldMineral()) {
            List<Recognition> updatedRecognitions = tensorFlow.getUpdatedRecognitions();
            if (shouldHandleUpdatedRecognitions(updatedRecognitions)) {
                telemetry.addLine("Looking for recognition");
                telemetry.update();
                handleUpdatedRecognitions(updatedRecognitions);
            }
        }
    }

    private boolean outOfTime() {
        return time != null && time.getTime() + 1000 < new Date().getTime();
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
            time = new Date();
            telemetry.addLine("Has not turned, turning 33");
            telemetry.update();
            hasTurned = true;
            driveSystem.turn(-33, 1);
        } else if (!doneSearching) {
            telemetry.addLine("Not done searching, turn -75");
            telemetry.update();
            driveSystem.turn(-20, 1);
            doneSearching = true;
            driveToGoldMineral();
        }
    }

    private void driveToGoldMineral() {
        telemetry.addLine("Driving to gold mineral");
        telemetry.update();
        if (!doneSearching)
        {
            driveSystem.turn(-90, 1);
        }
        driveSystem.driveToPositionInches(32, 1);
        hasDriven = true;
    }

    private boolean hasFoundGoldMineral(int goldMineralX) {
        return  goldMineralX != -1;
    }

    public void runSlideOut() {
        while(!slideSystem.isAtTop() && !isStopRequested()) {
            slideSystem.slideUp();
        }
        slideSystem.stop();
    }

    public int getCubePos() {
        if (!hasTurned) {
            return 0;
        } else if (hasTurned && !doneSearching) {
            return 1;
        }
        else {
            return 2;
        }
    }
}
