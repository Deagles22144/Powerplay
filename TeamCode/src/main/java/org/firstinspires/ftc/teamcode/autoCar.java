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
        encoderDriveP(0.25, -130, -130, -130, -130, 0.0025,2 );
        sleep(500);
        RotateP(127, 1, 2, 0.0165);
        elevatorHigh();
        tiltControl();
        sleep(1500);
        encoderDriveP(0.15, 14, 14,   14, 14, 0.015,2 );
        sleep(500);
        claw.setPosition(clawOpen);
        sleep(1000);
        elevatorGround();
        tiltControl();
        if (tagOfInterest == null || tagOfInterest.id == left) {
            //left code
            RotateP(0, 1,1.5,0.03);
            encoderDriveP(1 ,65,65,65,65,0.014,1);
            RotateP(-90, 1,2, 0.025);
            encoderDriveP(1, 55, 55, 55, 55, 0.025, 2);
        } else if (tagOfInterest.id == middle) {
            //middle code
            RotateP(180, 1, 2, 0.025);
            encoderDriveP(1, -55, -55, -55, -55, 0.025, 2);
        }
        else if (tagOfInterest.id == right) {
            //right code
            RotateP(180, 1, 2, 0.025);
            encoderDriveP(1, -60, -60, -60,-60, 0.025, 2);
            RotateP(90, 1, 2, 0.025);
            encoderDriveP(1, 50, 50, 50, 50.,0.02, 2);
            RotateP(180, 1, 2, 0.02);
            encoderDriveP(1, 20, 20, 20, 20, 0.25, 1.5);
        }
    }
}
