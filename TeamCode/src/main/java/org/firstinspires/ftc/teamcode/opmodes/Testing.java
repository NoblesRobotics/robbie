package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Testing",group="Linear OpMode")
public class Testing extends LinearOpMode {
    DcMotor tlMotor,trMotor,blMotor,brMotor,slideMotor;
    Servo turret,clawLeft,clawRight;
    DistanceSensor sensor;

    @Override
    public void runOpMode() {
        sensor = hardwareMap.get(DistanceSensor.class,"da");
        turret = hardwareMap.get(Servo.class, "sa");
        telemetry.addData("turret", turret.getPosition());
        telemetry.update();

        /*tlMotor = hardwareMap.get(DcMotor.class,"ma");
        trMotor = hardwareMap.get(DcMotor.class,"mc");
        blMotor = hardwareMap.get(DcMotor.class,"mb");
        brMotor = hardwareMap.get(DcMotor.class,"md");
        slideMotor = hardwareMap.get(DcMotor.class,"me");

        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        tlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        turret = hardwareMap.get(Servo.class, "sa");
        clawLeft = hardwareMap.get(Servo.class, "sb");
        clawRight = hardwareMap.get(Servo.class, "sc");

        waitForStart();

        /*while ( opModeIsActive() ) {
            telemetry.addData("distance",sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }*/

        while ( opModeIsActive() ) {
            if ( gamepad1.dpad_up ) turret.setPosition(turret.getPosition() + 0.001);
            else if ( gamepad1.dpad_down ) turret.setPosition(turret.getPosition() - 0.001);
            telemetry.addData("turret",turret.getPosition());
            if ( gamepad1.a ) clawLeft.setPosition(clawLeft.getPosition() + 0.001);
            else if ( gamepad1.b ) clawLeft.setPosition(clawLeft.getPosition() - 0.001);
            telemetry.addData("clawLeft",clawLeft.getPosition());
            if ( gamepad1.x ) clawRight.setPosition(clawRight.getPosition() + 0.001);
            else if ( gamepad1.y ) clawRight.setPosition(clawRight.getPosition() - 0.001);
            telemetry.addData("clawRight",clawRight.getPosition());
            telemetry.update();
        }

        /*while ( opModeIsActive() ) {
            if (gamepad1.a) slideMotor.setPower(1);
            else if (gamepad1.b) slideMotor.setPower(-1);
            else slideMotor.setPower(0);
        }*/

        /*telemetry.addData("tlMotor",tlMotor.getCurrentPosition());
        telemetry.update();
        tlMotor.setPower(1);
        sleep(1000);
        telemetry.addData("tlMotor",tlMotor.getCurrentPosition());
        telemetry.update();
        tlMotor.setPower(0);
        sleep(3000);

        telemetry.addData("trMotor",trMotor.getCurrentPosition());
        telemetry.update();
        trMotor.setPower(1);
        sleep(1000);
        telemetry.addData("trMotor",trMotor.getCurrentPosition());
        telemetry.update();
        trMotor.setPower(0);
        sleep(3000);

        telemetry.addData("blMotor",blMotor.getCurrentPosition());
        telemetry.update();
        blMotor.setPower(1);
        sleep(1000);
        telemetry.addData("blMotor",blMotor.getCurrentPosition());
        telemetry.update();
        blMotor.setPower(0);
        sleep(3000);

        telemetry.addData("brMotor",brMotor.getCurrentPosition());
        telemetry.update();
        brMotor.setPower(1);
        sleep(1000);
        telemetry.addData("brMotor",brMotor.getCurrentPosition());
        telemetry.update();
        brMotor.setPower(0);
        sleep(3000);*/
    }
}
