
//Encoder test teleOp. Not really sure where I got this.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder Test", group="Robot")
//@Disabled
public class EncoderTest extends LinearOpMode {

    private DcMotor         liftMotor;
    private boolean buttonPressed = false;
    private int targetPosition = 0;


    @Override
    public void runOpMode() {

        liftMotor  = hardwareMap.get(DcMotor.class, "lift_Motor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetPosition = liftMotor.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !buttonPressed) {
                buttonPressed = true;
                targetPosition += 500;
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.5);

                while (liftMotor.isBusy() && opModeIsActive()) {
                    telemetry.addData("Status", "Moving to target position...");
                    telemetry.update();
                }

                liftMotor.setPower(0);
            } else if (!gamepad1.a && buttonPressed) {
                buttonPressed = false;
            }

            telemetry.addData("Encoder Position", liftMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }
}
