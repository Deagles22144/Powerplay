package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="BlueLeft", group="Robot")
public class BlueLeft extends QRcode{


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
            encoderDriveP(1,-10,-10,-10,-10,0.019,2);
            RotateP(-90,1,2,0.019);
            encoderDriveP(1,55,55,55,55,0.019,2);
            RotateP(-180,1,2,0.019);
            encoderDriveP(1,60,60,60,60,0.019,2);
            RotateP(-180,1,2,0.019);
        } else if (tagOfInterest.id == middle) {
            //middle code
            encoderDriveP(1, -60, -60, -60, -60, 0.019, 1);
            RotateP(-180,1,2,0.019);
        }
         else if (tagOfInterest.id == right) {
            //right code
            encoderDriveP(1, -10, -10, -10, -10, 0.019, 2);
            RotateP(90, 1, 2, 0.019);
            encoderDriveP(1, 50, 50, 50, 50, 0.019, 2);
            RotateP(0, 1, 2, 0.019);
            encoderDriveP(1, -65, -65, -65, -65, 0.019, 2);
            RotateP(-180,1,2,0.019);
        }
    }
}
