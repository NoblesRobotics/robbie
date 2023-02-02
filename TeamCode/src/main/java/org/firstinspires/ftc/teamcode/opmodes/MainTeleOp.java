package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.Slide;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Main TeleOp",group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        CallbackManager callbackManager = new CallbackManager();

        Drivetrain drivetrain = new Drivetrain(this,0.5);
        Slide slide = new Slide(this);

        waitForStart();

        boolean isClawClosed = false;
        boolean isLiftScoring = false;
        boolean clawCommandNow;
        boolean clawCommandCaught = false;

        slide.setClawClosed(false);

        while ( opModeIsActive() ) {
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drivetrain.gamepadMove(drive,strafe,turn);

            clawCommandNow = false;
            if ( gamepad1.right_trigger > 0 ) {
                if ( ! clawCommandCaught ) {
                    clawCommandNow = true;
                    clawCommandCaught = true;
                }
            } else {
                clawCommandCaught = false;
            }

            if ( isLiftScoring ) {
                if ( clawCommandNow && isClawClosed ) {
                    slide.setClawClosed(false);
                    isClawClosed = false;
                } else if ( gamepad1.dpad_down ) {
                    slide.setClawClosed(true);
                    slide.setTurretBackwards(false);
                    callbackManager.add(new TimeCallback(0.5) {
                        @Override
                        void onFinished() {
                            slide.setLiftStage(Slide.GROUND_LIFT);
                            callbackManager.add(new Callback() {
                                @Override
                                public boolean update() {
                                    if ( slide.getActualLiftStage() <= Slide.GROUND_LIFT ) {
                                        slide.setClawClosed(false);
                                        return true;
                                    } else {
                                        return false;
                                    }
                                }
                            });
                        }
                    });
                    isClawClosed = false;
                    isLiftScoring = false;
                }
            } else { // lift not scoring
                if ( clawCommandNow ) {
                    if ( isClawClosed ) {
                        slide.setLiftStage(Slide.GROUND_LIFT);
                        callbackManager.add(new TimeCallback(0.5) {
                            @Override
                            void onFinished() {
                                slide.setClawClosed(false);
                            }
                        });
                        isClawClosed = false;
                    } else { // claw is not closed
                        slide.setClawClosed(true);
                        sleep(500);
                        slide.setLiftStage(Slide.MAX_SIZE_DRIVING_LIFT);
                        isClawClosed = true;
                    }
                } else if ( gamepad1.dpad_up && isClawClosed ) {
                    slide.setLiftStage(Slide.HIGH_LIFT);
                    callbackManager.add(new Callback() {
                        @Override
                        public boolean update() {
                            if ( slide.getActualLiftStage() >= Slide.HIGH_LIFT ) {
                                slide.setTurretBackwards(true);
                                return true;
                            } else {
                                return false;
                            }
                        }
                    });
                    isLiftScoring = true;
                }
            }

            if ( gamepad1.a ) {
                // Drive to stick w/o cone
                drivetrain.drive(-68,true);
                drivetrain.driveToDistance(3);
            } else if ( gamepad1.x ) {
                // Drive to stick w/ cone
                drivetrain.drive(-68, true);
                drivetrain.driveToDistance(2);
            } else if ( gamepad1.b ) {
                // Drive to triangle
                drivetrain.drive(65, true);
                drivetrain.rotateTo(0, 0.2);
            }

            /*if ( gamepad1.a ) slide.turret.setPosition(slide.turret.getPosition() + 0.001);
            else if ( gamepad1.b ) slide.turret.setPosition(slide.turret.getPosition() - 0.001);*/

            callbackManager.update();
        }
    }
}

interface Callback {
    // Return true on completion, false to run on next cycle
    boolean update();
}

abstract class TimeCallback implements Callback {
    ElapsedTime timer;
    double seconds;

    public TimeCallback(double seconds) {
        timer = new ElapsedTime();
        this.seconds = seconds;
    }

    @Override
    public boolean update() {
        if ( timer.seconds() >= seconds ) {
            onFinished();
            return true;
        } else {
            return false;
        }
    }

    abstract void onFinished();
}

class CallbackManager {
    List<Callback> callbacks = new ArrayList<>();

    public void add(Callback callback) {
        callbacks.add(callback);
    }

    public void update() {
        for ( int i = 0; i < callbacks.size(); i++ ) {
            if ( callbacks.get(i).update() ) {
                callbacks.remove(i);
                i--;
            }
        }
    }
}