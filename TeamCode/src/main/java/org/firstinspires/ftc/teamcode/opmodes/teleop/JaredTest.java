package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;


@TeleOp(name="nice", group="Linear OpMode")
public class JaredTest extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private AnalogInput sonar = null;
    private IMUSystem imuSystem = null;

    @Override
    public void runOpMode() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        sonar = hardwareMap.get(AnalogInput.class, "sonar");
        imuSystem = new IMUSystem(this);


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        double desiredHeading = 0.0;
        while (opModeIsActive()) {
<<<<<<< HEAD
            desiredHeading = driveWithGyroSystem(desiredHeading);
=======
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            double strafe = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);

            double rightPower = Range.clip(drive + turn, -1.0, 1.0) ;
            double leftPower = Range.clip(drive - turn, -1.0, 1.0) ;

            if(Math.abs(leftPower) < 0.12) {
                leftPower = 0.0;
            }
            if(Math.abs(rightPower) < 0.12) {
                rightPower = 0.0;
            }
            if(Math.abs(strafe) < 0.12) {
                strafe = 0.0;
            }

            double modifier;
            if(Math.abs(strafe) != 0.0) {
                modifier = (desiredHeading - imuSystem.getHeading()) / 25;
            } else {
                desiredHeading = imuSystem.getHeading();
                modifier = 0.0;
            }

            frontLeftDrive.setPower(leftPower - strafe + modifier);
            frontRightDrive.setPower(rightPower + strafe - modifier);
            backLeftDrive.setPower(leftPower + strafe + modifier);
            backRightDrive.setPower(rightPower - strafe - modifier);

            telemetry.addData("rightSonar", "%.3f v", (double) sonar.getVoltage());
            telemetry.addData("Heading", imuSystem.getHeading());
            telemetry.addData("Left stick", gamepad1.left_stick_y);
            telemetry.update();
>>>>>>> ec00586a2b5e7940e915281649f2b57c523fea9b
        }
    }

    public double driveWithGyroSystem(double desiredHeading) { // When using this in a loop, keep the return value stored, then call it again with the previous return value.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double strafe = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);

        double rightPower = Range.clip(drive + turn, -1.0, 1.0) ;
        double leftPower = Range.clip(drive - turn, -1.0, 1.0) ;

        if(Math.abs(leftPower) < 0.12) {
            leftPower = 0.0;
        }
        if(Math.abs(rightPower) < 0.12) {
            rightPower = 0.0;
        }
        if(Math.abs(strafe) < 0.12) {
            strafe = 0.0;
        }

        double modifier;
        if(Math.abs(strafe) != 0.0) {
            modifier = (desiredHeading - imuSystem.getHeading()) / 30;
        } else {
            desiredHeading = imuSystem.getHeading();
            modifier = 0.0;
        }

        frontLeftDrive.setPower(leftPower - strafe + modifier);
        frontRightDrive.setPower(rightPower + strafe - modifier);
        backLeftDrive.setPower(leftPower + strafe + modifier);
        backRightDrive.setPower(rightPower - strafe - modifier);

        telemetry.addData("Heading", imuSystem.getHeading());
        telemetry.addData("Left stick", gamepad1.left_stick_y);
        telemetry.update();

        return desiredHeading;
    }
}
