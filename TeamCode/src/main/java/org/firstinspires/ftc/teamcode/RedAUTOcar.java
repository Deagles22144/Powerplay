package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="RedAUTOcar", group="Robot")
public class RedAUTOcar extends QRcode{


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
