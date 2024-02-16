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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ParkBlueBackstageCloseEncoders", group="Robot")
@Disabled
public class ParkBlueBackstageCloseEncoders extends LinearOpMode {

    private DcMotor         leftfrontDrive   = null;
    private DcMotor         rightfrontDrive  = null;
    private DcMotor         leftbackDrive = null;
    private DcMotor         rightbackDrive = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 223 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {


        leftfrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftbackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightbackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                          leftfrontDrive.getCurrentPosition(),
                          rightfrontDrive.getCurrentPosition(),
                          leftbackDrive.getCurrentPosition(),
                          rightbackDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(0.6,  5,  5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -10, -10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newLeftTarget2;
        int newRightTarget;
        int newRightTarget2;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftfrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget2 = leftbackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightfrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget2 = rightbackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftfrontDrive.setTargetPosition(newLeftTarget);
            leftbackDrive.setTargetPosition(newLeftTarget2);
            rightfrontDrive.setTargetPosition(newRightTarget);
            rightbackDrive.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftfrontDrive.setPower(Math.abs(speed));
            rightfrontDrive.setPower(Math.abs(speed));
            leftbackDrive.setPower(Math.abs(speed));
            rightbackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //while (opModeIsActive() &&
                   //(runtime.seconds() < timeoutS) &&
                   //(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftbackDrive.isBusy() && rightbackDrive.isBusy())) {

                //telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Currently at",  " at %7d :%7d",
                                            //leftfrontDrive.getCurrentPosition(),
                                            //rightfrontDrive.getCurrentPosition(),
                                            //leftbackDrive.getCurrentPosition(),
                                            //rightbackDrive.getCurrentPosition());
                //telemetry.update();
            //}

            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);

            leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}
