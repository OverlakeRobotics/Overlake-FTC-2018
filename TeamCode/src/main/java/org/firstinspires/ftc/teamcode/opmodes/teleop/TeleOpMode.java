package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.controller.Handler;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.systems.arm.ArmDirection;
import org.firstinspires.ftc.teamcode.systems.arm.ArmState;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.flail.Flail;
import org.firstinspires.ftc.teamcode.systems.slide.SlideState;
import org.firstinspires.ftc.teamcode.systems.slide.SlideSystem;

/**
 * Created by idiot on 10/11/17.
 */
@TeleOp(name = "CompetitionTeleOp", group="TeleOp")
public class TeleOpMode extends BaseOpMode
{
    private Controller controller1;
    private Controller controller2;
    private MecanumDriveSystem driveSystem;
    private ArmSystem armSystem;
    private SlideSystem slideSystem;
    private Flail flail;

    private boolean slowDrive;

    public TeleOpMode() {
        msStuckDetectLoop = 1000000000;
    }

    @Override
    public void init()
    {
        this.controller1 = new Controller(gamepad1);
        this.controller2 = new Controller(gamepad2);
        armSystem = new ArmSystem(this);
        slideSystem = new SlideSystem(this);
        this.driveSystem = new MecanumDriveSystem(this);
        this.flail = new Flail(this);
        initButton();
        slowDrive = false;
    }

    public void initButton() {
        telemetry.addData("buttons", "initialize");
        telemetry.update();
        slideSystem.setOrigin(slideSystem.EncoderTop - slideSystem.getPosition());
        addWinchButton();
        addRotateButton();
        addFlailButton();
        addSlowDriveButton();
    }

    private void addWinchButton() {
        controller1.rightTrigger.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.WINCHING_TO_TOP);
            }
        };
        controller1.rightTrigger.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.IDLE);
            }
        };
        controller1.leftTrigger.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.WINCHING_TO_BOTTOM);
            }
        };
        controller1.leftTrigger.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.IDLE);
            }
        };
        controller2.rightTrigger.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.WINCHING_TO_TOP);
            }
        };
        controller2.rightTrigger.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.IDLE);
            }
        };
        controller2.leftTrigger.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.WINCHING_TO_BOTTOM);
            }
        };
        controller2.leftTrigger.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                slideSystem.setState(SlideState.IDLE);
            }
        };
    }

    private void addRotateButton() {
        controller1.dPadDown.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.ROTATING_BOTTOM);
            }
        };
        controller1.dPadDown.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller1.dPadUp.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.ROTATING_TOP);
            }
        };
        controller1.dPadUp.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller1.x.pressedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                armSystem.setState(ArmState.RELEASE_PIN);
            }
        };
        controller1.x.releasedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller1.y.pressedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                armSystem.setArmPin();
            }
        };
        controller1.y.releasedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller2.dPadDown.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.ROTATING_BOTTOM);
            }
        };
        controller2.dPadDown.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller2.dPadUp.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.ROTATING_TOP);
            }
        };
        controller2.dPadUp.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                armSystem.setState(ArmState.IDLE);
            }
        };
        controller2.x.pressedHandler = new Handler() {
            @Override
            public void invoke() throws Exception
            {
                latch();
            }
        };
    }

    private void addFlailButton() {
        controller1.rightBumper.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                flail.runForward();
            }
        };
        controller1.rightBumper.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                flail.stop();
            }
        };
        controller1.rightBumperShifted.pressedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                flail.runBackward();
            }
        };
        controller1.rightBumperShifted.releasedHandler = new Handler()
        {
            @Override
            public void invoke() throws Exception
            {
                flail.stop();
            }
        };
    }

    private void addSlowDriveButton() {
        controller1.leftStickButton.pressedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                slowDrive = true;
            }
        };
        controller1.leftStickButton.releasedHandler = new Handler() {
            @Override
            public void invoke() throws Exception {
                slowDrive = false;
            }
        };
    }

    @Override
    public void loop(){
        controller1.handle();
        controller2.handle();
        armSystem.run();
        slideSystem.run();

        float rx = controller1.gamepad.right_stick_x;
        float ry = controller1.gamepad.right_stick_y;
        float lx = controller1.gamepad.left_stick_x;
        float ly = controller1.gamepad.left_stick_y;

        driveSystem.mecanumDrive(rx, ry, lx, ly, slowDrive);
    }

    private void latch() {
        if (!armSystem.canLatch()) {
            armSystem.releaseArmPin();
            while (!armSystem.canLatch()) {
                armSystem.runMotors(armSystem.getDirectionToRun(armSystem.LatchPreparationVoltage));
            }
            armSystem.stop();
        } else {
            while(!armSystem.isLatched()) {
                if (armSystem.wheelsOnGround()) {
                    driveSystem.mecanumDriveXY(-0.5, 0);
                } else {
                    driveSystem.mecanumDriveXY(0,0);
                }
                armSystem.runMotors(ArmDirection.DOWN);
            }
            armSystem.setArmPin();
            sleep(500);
            armSystem.stop();
        }
    }

    private void sleep(int milis) {
        try
        {
            Thread.sleep(milis);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }
}
