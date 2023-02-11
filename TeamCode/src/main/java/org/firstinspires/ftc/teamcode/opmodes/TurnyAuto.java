package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.Slide;

@Autonomous(name="Turny Auto",group="Exercises")
public class TurnyAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode","initializing");
        telemetry.update();
        //SignalReader reader = new SignalReader(this);
        Drivetrain drivetrain = new Drivetrain(this,0.5);
        Slide slide = new Slide(this);
        telemetry.addData("Mode","waiting for start");
        telemetry.update();

        slide.setClawClosed(false);

        waitForStart();
        int signalStatus = 0;//reader.getSignalStatus(telemetry);
        telemetry.addData("Signal",signalStatus);
        if ( signalStatus == -1 ) signalStatus = 0;
        telemetry.addData("Mode","running");
        telemetry.update();

        /*while ( true ) {
            drivetrain.driveTurn(-16);
            drivetrain.driveToDistance(2.2);
            while (!gamepad1.a) idle();
            drivetrain.driveTurn(16);
            while (!gamepad1.a) idle();
        }*/

        while ( true ) {
            drivetrain.drive(-15);
            drivetrain.rotateTo(-90,0.2,2);
            drivetrain.drive(-10,0.25);
            drivetrain.driveToDistance(2.2);
            while (!gamepad1.a) idle();
            drivetrain.drive(6);
            drivetrain.rotateTo(0,0.2,2);
            drivetrain.drive(18);
            while (!gamepad1.a) idle();
        }
    }
}

