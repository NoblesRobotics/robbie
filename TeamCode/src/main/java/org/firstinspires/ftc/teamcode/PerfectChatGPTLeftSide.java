package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Perfect ChatGPT Left Side",group="Exercises")
public class PerfectChatGPTLeftSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,1);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.openClaw();
        sleep(1000);

        waitForStart();
        int signalStatus = reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        slide.closeClaw();
        slide.setLiftStage(Slide.OFF_GROUND_LIFT);
        drivetrain.drive(70);
        drivetrain.rotateTo(90,0.05);
        while ( !gamepad1.a ) {}
        drivetrain.drive(-13);
        //while ( !gamepad1.a ) {}
        drivetrain.drive(1);

        while ( !gamepad1.a ) {}
        // start score
        sleep(400);
        slide.setLiftStage(Slide.HIGH_LIFT);
        sleep(400);
        slide.setTurret(true);
        sleep(1800);
        slide.openClaw();
        sleep(1000);
        while ( !gamepad1.a ) {}
        slide.setTurret(false);
        slide.closeClaw();
        sleep(600);
        slide.setLiftStage(Slide.GROUND_LIFT);
        // end score

        /*//while ( !gamepad1.a ) {}
        drivetrain.drive(6);
        drivetrain.rotateTo(180);
        while ( !gamepad1.a ) {}
        drivetrain.drive(18);
        sleep(100);
        drivetrain.rotateTo(90,0.05);
        while ( !gamepad1.a ) {}

        slide.setLiftStage(Slide.CONE_5_LIFT);
        slide.openClaw();
        drivetrain.drive(26);
        slide.closeClaw();
        sleep(75);
        slide.setLiftStage(Slide.ABOVE_CONES);
        drivetrain.drive(-41); // MARK
        drivetrain.rotateTo(180,0.05);
        while ( !gamepad1.a ) {}
        drivetrain.drive(-12);

        // start score
        sleep(200);
        slide.setLiftStage(Slide.HIGH_LIFT);
        sleep(200);
        slide.setTurretExact(0.8);
        sleep(900);
        slide.openClaw();
        sleep(400);
        slide.setTurret(false);
        slide.closeClaw();
        sleep(300);
        slide.setLiftStage(Slide.GROUND_LIFT);
        // end score

        /*drivetrain.drive(13);
        drivetrain.rotateTo(-90);
        slide.setLiftStage(Slide.CONE_5_LIFT);
        slide.openClaw();
        drivetrain.drive(44);
        slide.closeClaw();
        sleep(75);
        slide.setLiftStage(Slide.ABOVE_CONES);
        drivetrain.drive(-41); // MARK
        drivetrain.rotateTo(-180,0.05);
        drivetrain.drive(-12);*/

        while ( !gamepad1.a ) {}
        drivetrain.drive(-12);
        while ( !gamepad1.a ) {}
        drivetrain.rotateTo(180);
        while ( !gamepad1.a ) {}
        drivetrain.drive(12);
        if ( signalStatus == 0 ) {
            drivetrain.rotateTo(90);
            drivetrain.drive(12);
        } else {
            drivetrain.rotateTo(270);
            drivetrain.drive(24);
        }

        /*drivetrain.drive(54); // from home forward
        drivetrain.rotateTo(-90); // turn towards cone stack
        drivetrain.drive(30); // drive to cone stack
        //while ( opModeIsActive() ) {}

        for ( int i = 0; i < 4; i++ ) {
            slide.closeClaw();
            sleep(75);
            slide.setLiftStage(Slide.OFF_GROUND_LIFT,false);

            drivetrain.drive(-42); // drive backwards towards stick
            drivetrain.rotateTo(-180); // turn (backwards) towards stick
            drivetrain.drive(-15); // drive backwards into stick

            /*drivetrain.drive(-32,false);
            drivetrain.smoothTurn(true);

            slide.setLiftStage(Slide.GROUND_LIFT,false);
            slide.openClaw();
            sleep(75);

            /*slide.setLiftStage(Slide.HIGH_LIFT);
            slide.setTurret(true);
            sleep(500);
            slide.openClaw();
            sleep(500);
            slide.closeClaw();
            slide.setTurret(false);
            sleep(500);
            slide.setLiftStage(Slide.GROUND_LIFT);

            drivetrain.drive(13);
            drivetrain.rotateTo(-90);
            drivetrain.drive(44);
        }*/
    }
}