package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Innovation",group="Linear OpMode")
public class Innovation extends LinearOpMode {
    DcMotor tlMotor,trMotor,blMotor,brMotor;

    @Override
    public void runOpMode() {
        tlMotor = hardwareMap.get(DcMotor.class,"ma");
        trMotor = hardwareMap.get(DcMotor.class,"mc");
        blMotor = hardwareMap.get(DcMotor.class,"mb");
        brMotor = hardwareMap.get(DcMotor.class,"md");

        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        tlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slide slide = new Slide(this);

        waitForStart();

        boolean clawClosed = false;
        boolean clawCommandCaught = false;
        int liftStage = 0;
        double turretStage = 0;
        while ( opModeIsActive() ) {
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x / 2;
            tlMotor.setPower(drive + strafe - turn);
            trMotor.setPower(drive - strafe + turn);
            blMotor.setPower(drive - strafe - turn);
            brMotor.setPower(drive + strafe + turn);

            if ( gamepad1.right_trigger > 0 ) {
                if ( ! clawCommandCaught ) {
                    clawClosed = ! clawClosed;
                    if ( clawClosed ) slide.closeClaw(true);
                    else slide.openClaw(true);
                }
                clawCommandCaught = true;
            } else {
                clawCommandCaught = false;
            }

            if ( gamepad1.b ) slide.turret.setPosition(slide.turret.getPosition() - 0.001);
            else if ( gamepad1.a ) slide.turret.setPosition(slide.turret.getPosition() + 0.001);
            telemetry.addData("turretStage",slide.turret.getPosition());

            if ( gamepad1.dpad_up ) {
                slide.setLiftStage(Slide.HIGH_LIFT);
                slide.setTurret(true);
            } else if ( gamepad1.dpad_down ) {
                slide.closeClaw();
                slide.setTurret(false);
                sleep(500);
                slide.setLiftStage(Slide.GROUND_LIFT);
                sleep(500);
                slide.openClaw();
            }
            if ( gamepad1.x ) slide.setLiftStage(slide.liftStage - 2);
            else if ( gamepad1.y ) slide.setLiftStage(slide.liftStage + 2);
            telemetry.addData("liftStage",slide.liftStage);
            telemetry.update();

            /*if ( gamepad1.a ) slide.turret.setPosition(0.2);
            else if ( gamepad1.b ) slide.turret.setPosition(0);
            else if ( gamepad1.x ) slide.clawLeft.setPosition(0.2);
            else if ( gamepad1.y ) slide.clawLeft.setPosition(0);
            else if ( gamepad1.left_bumper ) slide.clawRight.setPosition(0.2);
            else if ( gamepad1.right_bumper ) slide.clawRight.setPosition(0);*/
        }
    }
}
