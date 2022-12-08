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
    DcMotor tlMotor,trMotor,blMotor,brMotor,liftMotor;
    Servo clamp,turret;

    final double MAX_POS = 2840;

    final double TICKS_PER_REV = 537.6;
    final double MM_PER_REV = Math.PI * 96;
    final double MM_PER_INCH = 25.4;

    @Override
    public void runOpMode()
    {
        /*tlMotor = hardwareMap.get(DcMotor.class,"mb");
        trMotor = hardwareMap.get(DcMotor.class,"ma");
        blMotor = hardwareMap.get(DcMotor.class,"mc");
        brMotor = hardwareMap.get(DcMotor.class,"md");

        tlMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        trMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tlMotor.setPower(0.5);
        trMotor.setPower(0.5);
        blMotor.setPower(0.5);
        brMotor.setPower(0.5);

        waitForStart();

        drive(33);
        turnNinety(1);*/

        tlMotor = hardwareMap.get(DcMotor.class,"mb");
        trMotor = hardwareMap.get(DcMotor.class,"ma");
        blMotor = hardwareMap.get(DcMotor.class,"mc");
        brMotor = hardwareMap.get(DcMotor.class,"md");


        tlMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        trMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotor.class,"me");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0.5);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp = hardwareMap.get(Servo.class,"sa");
        turret = hardwareMap.get(Servo.class,"sb");
        clamp.setDirection(Servo.Direction.FORWARD);
        turret.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        double liftPosition = 0;
        double clampPosition = 0;
        double turretPosition = 0;
        while ( opModeIsActive() ) {
            /*double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            tlMotor.setPower(drive - strafe - turn);
            trMotor.setPower(drive + strafe + turn);
            blMotor.setPower(drive + strafe - turn);
            brMotor.setPower(drive - strafe + turn);*/

            liftPosition -= gamepad1.left_stick_y;
            if ( liftPosition < 0 ) liftPosition = 0;
            else if ( liftPosition > MAX_POS ) liftPosition = MAX_POS;
            if ( gamepad1.a ) liftPosition = MAX_POS;
            else if ( gamepad1.b ) liftPosition = 0;
            liftMotor.setTargetPosition((int) liftPosition);
            telemetry.addLine(String.valueOf(liftPosition));
            telemetry.update();

            if ( gamepad1.dpad_up ) turretPosition = 0.7;
            else if ( gamepad1.dpad_down ) turretPosition = 0;
            turret.setPosition(turretPosition);
            if ( gamepad1.x ) clampPosition = 0.5;
            else if ( gamepad1.y ) clampPosition = 0;
            clamp.setPosition(clampPosition);
        }
    }

    void drive(double amount) {
        moveMotors(amount,amount,amount,amount);
    }

    void turnNinety(double turn) {
        moveMotors(turn * .825,-turn * .825,turn * .825,-turn * .825);
    }

    void moveMotors(double tl,double tr,double bl,double br) {
        tlMotor.setTargetPosition((int) (tl * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV));
        trMotor.setTargetPosition((int) (tr * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV));
        blMotor.setTargetPosition((int) (bl * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV));
        brMotor.setTargetPosition((int) (br * MM_PER_INCH / MM_PER_REV * TICKS_PER_REV));

        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( tlMotor.isBusy() || trMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy() ) idle();

        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
