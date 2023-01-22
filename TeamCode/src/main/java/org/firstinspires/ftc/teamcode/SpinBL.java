package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SpinBL",group="Linear OpMode")
public class SpinBL extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor tlMotor = hardwareMap.get(DcMotor.class,"ma");
        DcMotor blMotor = hardwareMap.get(DcMotor.class,"mb");
        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while ( opModeIsActive() ) {
            tlMotor.setPower(gamepad1.left_stick_x);
            blMotor.setPower(gamepad1.left_stick_y);
        }
    }
}
