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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopMain3", group="Linear OpMode")
//@Disabled
public class TeleOpMain3 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor MotorLift;
    private DcMotor MotorCollector;
    private DcMotor MotorHook;
    private Servo ServoHook1;
    private Servo ServoHook2;
    private Servo ServoAirplane;
    private Servo ServoClamp;
    private Servo ServoWrist;
    private Servo ServoElbow;



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
        ServoHook1 = hardwareMap.get(Servo.class,"Servo_Hook_1");
        ServoHook2 = hardwareMap.get(Servo.class,"Servo_Hook_2");
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

        if (isStopRequested()) return;

        MotorCollectorState motorCollectorState = MotorCollectorState.off;


        while (opModeIsActive()) {

            double MotorHookPower = 0;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

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
                MotorCollector.setPower(-.4);
            }
            else if (motorCollectorState == MotorCollectorState.forward){
                MotorCollector.setPower(.4);
            }
            else {
                MotorCollector.setPower(0);
            }


            //Hook
            double hookspeed = gamepad2.left_trigger - gamepad2.right_trigger;

            if(gamepad2.dpad_up) {
                ServoHook1.setPosition(0);
                ServoHook2.setPosition(1);
            } else if (gamepad2.dpad_down) {
                ServoHook1.setPosition(0.3);
                ServoHook2.setPosition(0.7);
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

            }

            //Ready Position
            if(gamepad2.x) {
                ServoWrist.setPosition(0.4);
                ServoElbow.setPosition(0.4);
            }


            //Up position
            else if (gamepad2.y) {
                ServoWrist.setPosition(0.15);
                ServoElbow.setPosition(0.3);
            }


            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);


            MotorLift.setPower(lift);
            MotorHook.setPower(hookspeed);
            MotorHook.setPower(MotorHookPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

}
