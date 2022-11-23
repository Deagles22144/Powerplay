package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="autoCar", group="Robot")
public class autoCar extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()){
            QrScan();

        }


        waitForStart();
        arm.setPosition(0.02);
        encoderDriveP(1, -130, -130, -130, -130, 0.01,2 );
        sleep(500);
        RotateP(115, 1, 2, 0.03);
        elevatorHigh();
        tiltControl();
        sleep(1500);
        encoderDriveP(0.1, 10, 10,   10, 10, 0.025,2 );
        sleep(500);
        claw.setPosition(clawOpen);
        sleep(1000);
        elevatorGround();
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
