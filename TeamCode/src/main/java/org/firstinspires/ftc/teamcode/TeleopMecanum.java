package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleopMecanum", group="Robot")
public class TeleopMecanum extends RobotNew {

    @Override
    public void runOpMode() {
        super.runOpMode();

        //elapsedTime.time(TimeUnit.SECONDS);
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        claw.setPosition(clawOpen);
        //armPos(armReset);

        waitForStart();
        elapsedTime.reset();
        while (opModeIsActive() && !isStopRequested()) {


            double botHeading = -imu.getAngularOrientation().firstAngle /*- Math.PI*/;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
            double lfPower = (rotY + rotX + rx) /denominator;
            double lbPower = (rotY - rotX + rx) /denominator;
            double rfPower = (rotY - rotX - rx) /denominator;
            double rbPower = (rotY + rotX - rx) /denominator;


            drive.lf.setPower(/*y + x + rx*/lfPower);
            drive.lb.setPower(/*y - x + rx*/lbPower);
            drive.rf.setPower(/*y - x - rx*/rfPower);
            drive.rb.setPower(/*y + x - rx*/rbPower);

            if (gamepad1.circle){
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);

            }

            //double rt = gamepad1.right_trigger;
            boolean rightBumper = gamepad1.right_bumper;



            if (gamepad2.left_stick_y<0 /*&& arm0.getPosition()<0.11*/){
                armPos(arm0.getPosition() + 0.008);
            }
            else if (gamepad2.left_stick_y>0){
                armPos(arm0.getPosition() - 0.008);
            }
        /** Elevator Up**/
        if(gamepad2.right_trigger >= 0.1) {
            elevatorPower(1);
            elevatorTargetPosition(elevator0.getCurrentPosition()+60);
        }
        /***--------Elevator Down***/
        else if(gamepad2.left_trigger >= 0.1) {
                elevatorPower(1);
                elevatorTargetPosition(elevator0.getCurrentPosition() -60);
            }
            
//        if (gamepad2.left_trigger == 0 || gamepad2.right_trigger == 0) {
//            elevatorTargetPosition(elevator0.getCurrentPosition());
//        }


        /***Elevator HIGH***/
        if (gamepad2.dpad_up) {
            elapsedTime.reset();
            timerBrake = false;
            timerBrake1 = false;
            isGround = false;
            isLow = false;
            isMid = false;
            isHigh = true;
            elevatorHigh();

//            timerBrakeMid = false;
//            tiltPos(tiltHigh);
//            elevatorHighAuto();
        }
            if (elevator1.getCurrentPosition() >= elevatoeHighPos - 50 && elevator1.getCurrentPosition() <= elevatoeHighPos + 50 && isHigh) {
                armPos(armHigh);
                isHigh = false;
            }


//        if (timerBrake1 && elapsedTime.seconds() >= 1.2
//        ) {
//            timerBrake1 = false;
//            elevatorHigh();
//        }

            /***Elevator Mid***/

        if (gamepad2.dpad_left) {
            elapsedTime.reset();
//            timerBrakeMid = true;
            timerBrake = false;
            timerBrake1 = false;
            isGround = false;
            isLow = false;
            isMid = true;
            isHigh = false;
            elevatorMid();
//              elevatorMidAuto();
        }
        if (elevator1.getCurrentPosition() >= elevatorMiddlePos - 50 && elevator1.getCurrentPosition()<= elevatorMiddlePos + 50 && isMid) {
            armPos(armMid);
            isMid = false;
        }


//            if (timerBrakeMid && elapsedTime.seconds() >= 1) {
//                timerBrakeMid = false;
//                elevatorMid();
//            }

        /***-------Elevator LOW***/

        if (gamepad2.dpad_down) {
            timerBrake = false;
            timerBrakeMid = false;
            timerBrake1 = false;
            isGround = false;
            isLow = true;
            isMid = false;
            isHigh = false;
            elevatorLow();
        }

        /***-------Elevator Ground***/

        if (gamepad2.dpad_right) {
            elapsedTime.reset();
            timerBrake = false;
            timerBrake1 = true;
           // timerBrake2 = true;
            isGround  = true;
            isLow = false;
            isMid = false;
            isHigh = false;
//            timerBrakeMid = false;
            elevatorGround();
        }
    //    if (timerBrake1 && elapsedTime.seconds() >= 0.05) {
      //      claw.setPosition(clawClose);
        //    timerBrake1 = false;
        //}

        if (timerBrake1 && elapsedTime.seconds() >= 0.7) {
                claw.setPosition(clawOpen);
                timerBrake1 = false;
        }


//            if (elevator0.getCurrentPosition() <= elevatorMiddlePos/2 && isGround) {
//            armPos(armGround);
//            tiltPos(tiltGround);
//            isGround = false;
//        }


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



        if (gamepad2.right_stick_y<0 /*&& tilt0.getPosition()<0.1*/){
           // tiltPos(tilt0.getPosition()+0.004);
        }
        else if (gamepad2.right_stick_y>0){
          //  tiltPos(tilt0.getPosition()-0.004);
        }

        if (rightBumper) {
            ToggleClaw();
        }

        if (elapsedTime.seconds() >= 0.3 && timerBrake && claw.getPosition() == clawClose) {
            armPos(armLoadCone);
        }

        if (elapsedTime.seconds() >= 0.4 && timerBrake && claw.getPosition() == clawOpen) {
            claw.setPosition(clawClose);
            close = false;
        }

        if (elapsedTime.seconds() >= 0.3 && timerBrake && claw.getPosition() == clawClose && arm0.getPosition() == armHigh) {
            armPos(armHigh);
        }

        if (!rightBumper) {
            wasPressed = true;
        }

//        if(!isGround && claw.getPosition() == clawOpen) {
//            armPos(0.77);
//        }

        telemetry.addLine("claw position: " + claw.getPosition());
        telemetry.addLine("elevator is at: " + elevator0.getCurrentPosition());
        telemetry.addLine("arm0 is at: " + arm0.getPosition());
        telemetry.addLine("arm1 is at: " + arm1.getPosition());
        telemetry.addLine("timer: " + elapsedTime.seconds());
        telemetry.addLine("timebreake is: " + timerBrake);
        telemetry.addLine("timebreake1 is: " + timerBrake1);
        // telemetry.addLine("heading: " + botHeading);
        telemetry.update();
      }
    }
}
