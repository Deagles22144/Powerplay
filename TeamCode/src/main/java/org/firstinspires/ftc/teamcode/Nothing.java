package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Nothing", group="Robot")
public class Nothing extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }


        waitForStart();

        arm.setPosition(0.02);

        if (tagOfInterest == null || tagOfInterest.id == left) {
            //left code
            encoderDriveP(1, 100, 100, 100, 100, 0.025, 1.5);
            RotateP(180, 1, 2, 0.019);
            encoderDriveP(1, 100, 100, 100, 100, 0.25,2);
            RotateP(0, 1, 2, 0.25);

        } else if (tagOfInterest.id == middle) {
            //middle code
            encoderDriveP(1, 58, 58, 58, 58, 0.025, 1.5);
            RotateP(10, 1, 1.5, 0.019);
            elevatorHigh();
            tiltControl();
            sleep(1000);
            claw.setPosition(clawOpen);
            elevatorGround();
            tiltControl();
            sleep(1000);
            claw.setPosition(clawClose);
        }
        else if (tagOfInterest.id == right) {
            //right code
            RotateP(75, 1, 2, 0.019);
            elevatorHigh();
            tiltControl();
            sleep(1000);
            encoderDriveP(1, 8, 8, 8, 8, 0.019,2 );
            claw.setPosition(clawOpen);
        }
    }
}
