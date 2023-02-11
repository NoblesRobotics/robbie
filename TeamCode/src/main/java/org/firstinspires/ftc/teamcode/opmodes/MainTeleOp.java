package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.callback.Callback;
import org.firstinspires.ftc.teamcode.callback.CallbackManager;
import org.firstinspires.ftc.teamcode.callback.DriveCallback;
import org.firstinspires.ftc.teamcode.control.Drivetrain;
import org.firstinspires.ftc.teamcode.control.Slide;
import org.firstinspires.ftc.teamcode.callback.TimeCallback;

@TeleOp(name="Main TeleOp",group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        CallbackManager callbackManager = new CallbackManager();

        Drivetrain drivetrain = new Drivetrain(this,0.5);
        drivetrain.setGlobalAngle(90);
        Slide slide = new Slide(this);

        waitForStart();

        final boolean[] isClawClosed = {false};
        final boolean[] isLiftScoring = {false};
        boolean clawCommandNow;
        boolean clawCommandCaught = false;
        boolean leftCommandCaught = false;
        boolean rightCommandCaught = false;
        final int[] liftSequenceIndex = {0};
        final boolean[] gamepadActive = {true};

        slide.setClawClosed(false);

        while ( opModeIsActive() ) {
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x / 2;
            if (gamepadActive[0]) drivetrain.gamepadMove(drive,strafe,turn);

            clawCommandNow = false;
            if ( gamepad1.right_trigger > 0 ) {
                if ( ! clawCommandCaught ) {
                    clawCommandNow = true;
                    clawCommandCaught = true;
                }
            } else {
                clawCommandCaught = false;
            }

            /*if ( gamepad1.dpad_left ) {
                if ( ! leftCommandCaught ) {
                    if ( liftSequenceIndex[0] > 0 ) liftSequenceIndex[0]--;
                    slide.setLiftStage(Slide.LIFT_SEQUENCE[liftSequenceIndex[0]]);
                    leftCommandCaught = true;
                }
            } else {
                leftCommandCaught = false;
            }
            if ( gamepad1.dpad_right ) {
                if ( ! rightCommandCaught ) {
                    if ( liftSequenceIndex[0] < Slide.LIFT_SEQUENCE.length - 1 ) liftSequenceIndex[0]++;
                    slide.setLiftStage(Slide.LIFT_SEQUENCE[liftSequenceIndex[0]]);
                    rightCommandCaught = true;
                }
            } else {
                rightCommandCaught = false;
            }*/

            if ( gamepad1.dpad_left ) slide.setLiftStage(Slide.MEDIUM_LIFT);
            else if ( gamepad1.dpad_right ) slide.setLiftStage(Slide.LOW_LIFT);

            if (isLiftScoring[0]) {
                if ( clawCommandNow && isClawClosed[0]) {
                    slide.setClawClosed(false);
                    isClawClosed[0] = false;
                } else if ( gamepad1.dpad_down ) {
                    slide.setClawClosed(true);
                    slide.setTurret(Slide.FORWARDS_TURRET);
                    callbackManager.add(new TimeCallback(0.5) {
                        @Override
                        public void onFinished() {
                            liftSequenceIndex[0] = 0;
                            slide.setLiftStage(Slide.GROUND_LIFT);
                            callbackManager.add(new Callback() {
                                @Override
                                public boolean update() {
                                    if (slide.getActualLiftStage() <= Slide.GROUND_LIFT) {
                                        slide.setClawClosed(false);
                                        return true;
                                    } else {
                                        return false;
                                    }
                                }
                            });
                        }
                    });
                    isClawClosed[0] = false;
                    isLiftScoring[0] = false;
                }
            } else { // lift not scoring
                if ( clawCommandNow ) {
                    if (isClawClosed[0]) {
                        slide.setTurret(Slide.FORWARDS_TURRET);
                        sleep(300);
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
                        isClawClosed[0] = false;
                    } else { // claw is not closed
                        slide.setClawClosed(true);
                        sleep(500);
                        slide.setLiftStage(Slide.MAX_SAFE_DRIVING_LIFT);
                        callbackManager.add(new Callback() {
                            @Override
                            public boolean update() {
                                if ( slide.getActualLiftStage() >= Slide.MAX_SAFE_DRIVING_LIFT ) {
                                    slide.setTurret(Slide.SIDE_TURRET);
                                    return true;
                                } else {
                                    return false;
                                }
                            }
                        });
                        isClawClosed[0] = true;
                    }
                } else if ( gamepad1.dpad_up && isClawClosed[0]) {
                    slide.setLiftStage(Slide.HIGH_LIFT);
                    callbackManager.add(new Callback() {
                        @Override
                        public boolean update() {
                            if ( slide.getActualLiftStage() >= Slide.HIGH_LIFT ) {
                                slide.setTurret(Slide.BACKWARDS_TURRET);
                                return true;
                            } else {
                                return false;
                            }
                        }
                    });
                    isLiftScoring[0] = true;
                }
            }

            if ( gamepad1.a && gamepadActive[0] ) {
                // Drive to stick w/ cone
                drivetrain.setPower(0.5);
                slide.setClawClosed(true);
                sleep(500);
                slide.setLiftStage(Slide.HIGH_LIFT);
                callbackManager.add(new Callback() {
                    @Override
                    public boolean update() {
                        if ( slide.getActualLiftStage() >= Slide.MIN_SAFE_TURNING_LIFT ) {
                            slide.setTurret(Slide.SIDE_TURRET);
                            return true;
                        } else {
                            return false;
                        }
                    }
                });
                drivetrain.rotateTo(0, 0.2);
                gamepadActive[0] = false;
                drivetrain.driveAsync(-36,callbackManager,new DriveCallback() {
                    @Override
                    public void onFinished() {
                        gamepadActive[0] = true;
                        isClawClosed[0] = true;
                        isLiftScoring[0] = true;
                        slide.setTurret(Slide.BACKWARDS_TURRET);
                        drivetrain.rotateTo(0, 0.2);
                        drivetrain.driveToDistance(1.7);
                        drivetrain.driveToDistance(1.7);
                    }
                });
            } else if ( gamepad1.x && gamepadActive[0] ) {
                // Drive to stick w/o cone
                drivetrain.setPower(0.5);
                slide.setClawClosed(true);
                sleep(500);
                slide.setLiftStage(Slide.HIGH_LIFT);
                callbackManager.add(new Callback() {
                    @Override
                    public boolean update() {
                        if ( slide.getActualLiftStage() >= Slide.MIN_SAFE_TURNING_LIFT ) {
                            slide.setTurret(Slide.SIDE_TURRET);
                            return true;
                        } else {
                            return false;
                        }
                    }
                });
                drivetrain.rotateTo(0, 0.2);
                gamepadActive[0] = false;
                drivetrain.driveAsync(-36,callbackManager,new DriveCallback() {
                    @Override
                    public void onFinished() {
                        gamepadActive[0] = true;
                        isClawClosed[0] = true;
                        isLiftScoring[0] = true;
                        slide.setTurret(Slide.BACKWARDS_TURRET);
                        drivetrain.rotateTo(0, 0.2);
                        drivetrain.driveToDistance(3);
                        drivetrain.driveToDistance(3);
                    }
                });
            } else if ( gamepad1.b && gamepadActive[0] ) {
                // Drive to triangle
                drivetrain.setPower(0.25);
                slide.setClawClosed(false);
                sleep(500);
                slide.setClawClosed(true);
                slide.setTurret(Slide.FORWARDS_TURRET);
                callbackManager.add(new Callback() {
                    @Override
                    public boolean update() {
                        if ( slide.getActualTurret() <= Slide.SIDE_TURRET ) {
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
                            return true;
                        } else {
                            return false;
                        }
                    }
                });
                drivetrain.rotateTo(0, 0.2);
                gamepadActive[0] = false;
                drivetrain.driveAsync(31, callbackManager, new DriveCallback() {
                    @Override
                    public void onFinished() {
                        gamepadActive[0] = true;
                        isClawClosed[0] = false;
                        isLiftScoring[0] = false;
                        drivetrain.rotateTo(0, 0.2);
                    }
                });
            }

            /*if ( gamepad1.x ) slide.turret.setPosition(slide.turret.getPosition() + 0.001);
            else if ( gamepad1.y ) slide.turret.setPosition(slide.turret.getPosition() - 0.001);
            telemetry.addData("turret",slide.turret.getPosition());
            telemetry.update();*/

            /*if ( gamepad1.x ) slide.setLiftStage(slide.getActualLiftStage() + 4);
            else if ( gamepad1.y ) slide.setLiftStage(slide.getActualLiftStage() - 4);
            telemetry.addData("slide",slide.getActualLiftStage());
            telemetry.update();*/

            callbackManager.update();
        }
    }
}