package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="AUTOcarFirst", group="Robot")
public class AUTOcarFirst extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()){
            QrScan();

        }
        waitForStart();

        arm.setPosition(0.02);
        encoderDriveP(0.25, -125, -125, -125, -125, 0.003,5 );
        sleep(500);
        RotateP(140, 1, 3.5, 0.0168);
        //Elevator High
        elevator.setTargetPosition(high);
        arm.setPosition(0.081);
        tilt.setPosition(0.018);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
        sleep(1500);
        encoderDriveP(0.075, 17, 17,   17, 17, 0.01,2.5  );
        sleep(500);
        claw.setPosition(clawOpen);
        sleep(1000);
        elevatorGround();
        tiltControl();

        sleep(500);
        if (tagOfInterest == null || tagOfInterest.id == left) {
            //left code

        } else if (tagOfInterest.id == middle) {
            //middle code

        }
        else if (tagOfInterest.id == right) {
            //right code

        }
    }
}

