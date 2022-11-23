package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleopMecanum", group="Robot")
public class TeleopMecanum extends RobotNew {

    @Override
    public void runOpMode() {
        super.runOpMode();

        //elapsedTime.time(TimeUnit.SECONDS);
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        claw.setPosition(clawOpen);

        waitForStart();
        elapsedTime.reset();
        while (opModeIsActive() && !isStopRequested()) {


            double botHeading = Math.toRadians(-imu.getAngularOrientation().firstAngle) - Math.PI;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
            double lfPower = (rotY + rotX + rx) /denominator;
            double lbPower = (rotY - rotX + rx) /denominator;
            double rfPower = (rotY - rotX - rx) /denominator;
            double rbPower = (rotY + rotX - rx) /denominator;


            lf.setPower(/*y + x + rx*/lfPower);
            lb.setPower(/*y - x + rx*/lbPower);
            rf.setPower(/*y - x - rx*/rfPower);
            rb.setPower(/*y + x - rx*/rbPower);

            //double rt = gamepad1.right_trigger;
            boolean rightBumper = gamepad1.right_bumper;


            /*if (gamepad1.circle){
                parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imu.initialize(parameters);
            }
*/
            tiltControl();

            if (gamepad2.left_stick_y<0 && arm.getPosition()<0.11){
                arm.setPosition(arm.getPosition() + 0.0004);
            }
            else if (gamepad2.left_stick_y>0){
                arm.setPosition(arm.getPosition() - 0.0004);
            }


        if (gamepad2.dpad_up) {
            timerBrake = false;
            elevatorHigh();

        }

        if (gamepad2.dpad_left) {
            timerBrake = false;
            elevatorMid();
        }

        if (gamepad2.dpad_down) {
            timerBrake = false;
            elevatorLow();
        }

        if (gamepad2.dpad_right) {
            timerBrake = false;
            elevatorGround();
        }
        if (gamepad1.right_trigger > 0.5) {
            y = -gamepad1.left_stick_y / 2;
            x = gamepad1.left_stick_x / 2;
            rx = gamepad1.right_stick_x / 2;
        }
            else if (gamepad1.left_trigger > 0.5) {
                y = -gamepad1.left_stick_y / 10;
                x = gamepad1.left_stick_x / 10;
                rx = gamepad1.right_stick_x / 10;
            }
        else {
            y = -gamepad1.left_stick_y ;
            x = gamepad1.left_stick_x ;
            rx = gamepad1.right_stick_x ;

        }

        if (gamepad2.right_stick_y<0 && tilt.getPosition()<0.1){
            tilt.setPosition(tilt.getPosition()+0.0004);
        }
        else if (gamepad2.right_stick_y>0){
            tilt.setPosition(tilt.getPosition()-0.0004);
        }

        if (rightBumper) {
            ToggleClaw();
        }
        if (elapsedTime.seconds() >= 0.45 && timerBrake && claw.getPosition() == clawClose) {
            arm.setPosition(0.02);
        }

        if (!rightBumper) {
            wasPressed = true;
        }

        telemetry.addLine("elevator is at: " + elevator.getCurrentPosition());
        telemetry.addLine("Arm is at: " + arm.getPosition());
        telemetry.addLine("timer: " + elapsedTime.seconds());
        telemetry.addLine("timebreake is: " + timerBrake);
       // telemetry.addLine("heading: " + botHeading);
        telemetry.update();
      }
    }
}
