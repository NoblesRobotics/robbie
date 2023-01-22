package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Simplest Auto",group="Exercises")
public class SimplestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,1);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        waitForStart();
        int signalStatus = reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        drivetrain.drive(42);
        drivetrain.drive(-12); // MARK

        if ( signalStatus == 0 || signalStatus == 2 ) {
            if ( signalStatus == 2 ) drivetrain.rotateTo(-90);
            else drivetrain.rotateTo(90);
            drivetrain.drive(24);
        }
    }
}