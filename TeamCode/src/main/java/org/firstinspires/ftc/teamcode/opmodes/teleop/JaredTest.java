package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;


@TeleOp(name="jjjjjjjareada", group="Linear Opmode")
public class JaredTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private IMUSystem imuSystem = null;
    private double desiredHeading;
    private final int loops = 10; // bröther
    private double frontLeftModifier, frontRightModifier, backLeftModifier, backRightModifier = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        imuSystem = new IMUSystem(this);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        desiredHeading = imuSystem.getHeading();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x*1.2;

            double strafe = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
//            frontLeftDrive.setPower(-strafe);
//            backRightDrive.setPower(-strafe);
//            frontRightDrive.setPower(strafe);

//
            backLeftDrive.setPower(strafe);
            rightPower = Range.clip(drive + turn, -1.0, 1.0) ;
            leftPower = Range.clip(drive - turn, -1.0, 1.0) ;

            if(Math.abs(leftPower) < 0.13) {
                leftPower = 0;
            }
            if(Math.abs(rightPower) < 0.13) {
                rightPower = 0;
            }
            if(Math.abs(strafe) < 0.13) {
                strafe = 0;
            }

            double modifier = 0.0;
            if(Math.abs(strafe) != 0.0) {
                modifier = ((desiredHeading - imuSystem.getHeading()) / 20) * Math.abs(strafe);
            } else {
                desiredHeading = imuSystem.getHeading();
                modifier = 0.0;
            }
            frontLeftDrive.setPower(leftPower - strafe + modifier);
            frontRightDrive.setPower(rightPower + strafe - modifier);
            backLeftDrive.setPower(leftPower + strafe + modifier);
            backRightDrive.setPower(rightPower - strafe - modifier);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading", imuSystem.getHeading());
            telemetry.addData("Left stick", gamepad1.left_stick_y);
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
