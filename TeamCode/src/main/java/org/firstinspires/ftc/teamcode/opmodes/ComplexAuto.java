package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.SignalReader;
import org.firstinspires.ftc.teamcode.control.Slide;

@Disabled
@TeleOp(name="Complex Auto",group="Exercises")
public class ComplexAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        //SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,0.5);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.setClawClosed(true);

        waitForStart();
        int signalStatus = 2;//reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        drivetrain.drive(78);
        while ( ! gamepad1.a ) idle();
        drivetrain.drive(-12);
        while ( ! gamepad1.a ) idle();
        //slide.setLiftStage(Slide.MAX_SIZE_DRIVING_LIFT);
        drivetrain.rotateTo(90,0.2,2);
        while ( ! gamepad1.a ) idle();
        drivetrain.drive(-8);
        while ( ! gamepad1.a ) idle();
        drivetrain.driveToDistance(2.2);
        while ( ! gamepad1.a ) idle();

        /*slide.setLiftStage(Slide.HIGH_LIFT);
        while ( slide.getActualLiftStage() < Slide.HIGH_LIFT ) idle();
        slide.setTurretBackwards(true);
        sleep(500);
        slide.setClawClosed(false);
        sleep(500);
        slide.setClawClosed(true);
        slide.setTurretBackwards(false);
        sleep(500);
        slide.setLiftStage(Slide.GROUND_LIFT);
        while ( slide.getActualLiftStage() > Slide.MAX_SIZE_DRIVING_LIFT ) idle();*/

        drivetrain.drive(8);
        while ( ! gamepad1.a ) idle();
        drivetrain.rotateTo(0,0.2,2);
        while ( ! gamepad1.a ) idle();
        drivetrain.drive(-12);
        while ( ! gamepad1.a ) idle();

        if ( signalStatus == 0 || signalStatus == 2 ) {
            if ( signalStatus == 2 ) drivetrain.rotateTo(-90,0.2,2);
            else drivetrain.rotateTo(90,0.2,2);
            while ( ! gamepad1.a ) idle();
            drivetrain.drive(24);
        }
    }
}

