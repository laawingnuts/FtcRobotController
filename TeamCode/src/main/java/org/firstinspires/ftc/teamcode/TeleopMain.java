/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleopMain", group="Linear OpMode")
//Disabled
public class TeleopMain extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor MotorLift;
    private DcMotor MotorCollector;
    private DcMotor MotorHook;
    private Servo ServoHook;
    private Servo ServoAirplane;
    private Servo ServoClamp;
    private Servo ServoWrist;
    private Servo ServoElbow;

    static final double     COUNTS_PER_MOTOR_REV    = 223 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    enum MotorCollectorState  {
        forward, backward, off;
    };


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        MotorLift = hardwareMap.get(DcMotor.class, "Motor_Lift");
        MotorCollector = hardwareMap.get(DcMotor.class, "Motor_Collector");
        MotorHook = hardwareMap.get(DcMotor.class, "Motor_Hook");
        ServoHook = hardwareMap.get(Servo.class,"Servo_Hook");
        ServoAirplane = hardwareMap.get(Servo.class, "Servo_Airplane");
        ServoClamp = hardwareMap.get(Servo.class, "Servo_Clamp");
        ServoWrist = hardwareMap.get(Servo.class, "Servo_Wrist");
        ServoElbow = hardwareMap.get(Servo.class, "Servo_Elbow");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime mStateTime = new ElapsedTime();
        int v_state = 0;

        waitForStart();
        runtime.reset();


        MotorCollectorState motorCollectorState = MotorCollectorState.off;


        while (opModeIsActive()) {

            double MotorHookPower = 0;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            //double max;


            //double axial   = -gamepad1.left_stick_y;
            //double lateral =  gamepad1.left_stick_x;
            //double yaw     =  gamepad1.right_stick_x;


            //double leftFrontPower  = axial + lateral + yaw;
            //double rightFrontPower = axial - lateral - yaw;
            //double leftBackPower   = axial - lateral + yaw;
            //double rightBackPower  = axial + lateral - yaw;

            //max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            //max = Math.max(max, Math.abs(leftBackPower));
            //max = Math.max(max, Math.abs(rightBackPower));

            //if (max > 1.) {
                //leftFrontPower  /= max;
                //rightFrontPower /= max;
                //leftBackPower   /= max;
                //rightBackPower  /= max;
            //}

            if (gamepad1.a) {
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            } else if (gamepad1.b) {
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }

            //Buttons Used:
            // Gamepad1- leftstick, rightstick, right trigger, left trigger, dpad up, dpad down, x, a, b
            // Gamepad2- rightstick, left trigger, right trigger, left bumper, right bumper, x, y, a, b

            //Lift
            double lift = gamepad2.right_stick_y - gamepad2.left_stick_y;

            //Collector
            if(gamepad1.left_bumper) {
                motorCollectorState = MotorCollectorState.forward;

            }else if (gamepad1.right_bumper) {
                motorCollectorState = MotorCollectorState.backward;
            } else if (gamepad1.x) {
                motorCollectorState = MotorCollectorState.off;
            }

            if (motorCollectorState == MotorCollectorState.backward) {
                MotorCollector.setPower(-1);
            }
            else if (motorCollectorState == MotorCollectorState.forward){
                MotorCollector.setPower(1);
            }
            else {
                MotorCollector.setPower(0);
            }


            //Hook
            double hookspeed = gamepad2.left_trigger - gamepad2.right_trigger;

            if(gamepad2.dpad_up) {
                ServoHook.setPosition(-0.5);
            } else if (gamepad2.dpad_down) {
                ServoHook.setPosition(0.35);

            }

            //Airplane
            if(gamepad1.dpad_up) {
                ServoAirplane.setPosition(0.2);
            }else if (gamepad1.dpad_down) {
                ServoAirplane.setPosition(1);
            }

            //Servo for grabber
            if(gamepad2.left_bumper) {
                ServoClamp.setPosition(0.5);
            }else if (gamepad2.right_bumper) {
                ServoClamp.setPosition(0.1);
            }

            //wrist and elbow

            //down position
            if (gamepad2.a)
            {
                ServoWrist.setPosition(0.23);
                ServoElbow.setPosition(0.15);
            }

            //Travel position
            if(gamepad2.b) {
                ServoWrist.setPosition(0.27);
                ServoElbow.setPosition(0.18);
                sleep(2000);
                encoderDrive(0.6,  5,  5, 5.0);
                sleep(1000);

            }

            //Ready Position
            //if(gamepad2.x) {
                //ServoWrist.setPosition(0.4);
                //ServoElbow.setPosition(0.4);
            //}

            if(gamepad2.x)  {

                switch (v_state){

                    case 0:

                        ServoWrist.setPosition(0.27);
                        ServoElbow.setPosition(0.18);
                        mStateTime.reset();
                        v_state++;
                        break;
                    case 1:
                        if (mStateTime.time() >= 0.5) {
                            ServoWrist.setPosition(0.4);
                            ServoElbow.setPosition(0.4);
                            v_state++;
                        }
                }
            }


            //Up position
            else if (gamepad2.y) {
                ServoWrist.setPosition(-0.1);
                ServoElbow.setPosition(0.15);
            }


            //leftFrontDrive.setPower(leftFrontPower);
            //rightFrontDrive.setPower(rightFrontPower);
            //leftBackDrive.setPower(leftBackPower);
            //rightBackDrive.setPower(rightBackPower);
            MotorLift.setPower(lift);
            MotorHook.setPower(hookspeed);
            MotorHook.setPower(MotorHookPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newMotorLiftTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMotorLiftTarget = MotorLift.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            MotorLift.setTargetPosition(newMotorLiftTarget);


            // Turn On RUN_TO_POSITION
            MotorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            MotorLift.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (MotorLift.isBusy())) {

            }

            // Stop all motion;
            MotorLift.setPower(0);

            // Turn off RUN_TO_POSITION
            MotorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
