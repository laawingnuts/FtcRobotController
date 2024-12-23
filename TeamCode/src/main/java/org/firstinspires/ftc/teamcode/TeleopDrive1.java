/* Copyright (c) 2021 FIRST. All rights reserved.
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

//TeleOp Drive with Field Centric controls

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class TeleopDrive1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorLift;
    private DcMotor MotorCollector;
    private DcMotor MotorHook;
    private Servo ServoHook1;
    private Servo ServoHook2;
    private Servo ServoAirplane;
    private Servo ServoClamp;
    private Servo ServoWrist;
    private Servo ServoElbow;
    private CRServo TestServo;
    enum MotorCollectorState  {
        forward, backward, off;
    };
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");

        MotorLift = hardwareMap.get(DcMotor.class, "Motor_Lift");
        MotorCollector = hardwareMap.get(DcMotor.class, "Motor_Collector");
        MotorHook = hardwareMap.get(DcMotor.class, "Motor_Hook");
        ServoHook1 = hardwareMap.get(Servo.class,"Servo_Hook_1");
        ServoHook2 = hardwareMap.get(Servo.class,"Servo_Hook_2");
        ServoAirplane = hardwareMap.get(Servo.class, "Servo_Airplane");
        ServoClamp = hardwareMap.get(Servo.class, "Servo_Clamp");
        ServoWrist = hardwareMap.get(Servo.class, "Servo_Wrist");
        ServoElbow = hardwareMap.get(Servo.class, "Servo_Elbow");
        TestServo = hardwareMap.get(CRServo.class, "Test_Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        MotorCollectorState motorCollectorState = MotorCollectorState.off;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

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
            double MotorHookPower = 0;
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

            //Lift and Grabber

            //wrist and elbow
            //Servo for grabber
            if(gamepad2.left_bumper) { // open
                ServoClamp.setPosition(0.5);
            }else if (gamepad2.right_bumper) { // closed
                ServoClamp.setPosition(0.1);
            }

            //down position
            if (gamepad2.a)
            {
                ServoWrist.setPosition(0.02);
                ServoElbow.setPosition(0.08);

            }

            //Travel position
            if(gamepad2.b) {
                ServoWrist.setPosition(0.22);
                ServoElbow.setPosition(0.14);

            }

            //Ready Position
            if(gamepad2.x) {
                ServoWrist.setPosition(0.33);
                ServoElbow.setPosition(0.3);
            }


            //Up position
            else if (gamepad2.y) {
                ServoWrist.setPosition(0.15);
                ServoElbow.setPosition(0.3);
            }

            if (gamepad1.a) {
                TestServo.setPower(1);
            } else if (gamepad1.b) {
                TestServo.setPower(-1);
            } else if (gamepad1.x) {
                TestServo.setPower(0);
            }

            MotorLift.setPower(lift);
            MotorHook.setPower(hookspeed);
            MotorHook.setPower(MotorHookPower);

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

        }
    }
}

