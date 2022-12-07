package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="AUTOcarFirst", group="Robot")
public class AUTOcarFirst extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()){
            QrScan();

        }
        waitForStart();
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(35.6,-67.2, Math.toRadians(-31)), Math.toRadians(91))
                .splineToSplineHeading(new Pose2d(40,-12, Math.toRadians(-180)), Math.toRadians(-31))
                .build();
        drive.followTrajectory(traj);
        elevatorHigh();
        tilt.setPosition(0.2);
        sleep(200);
        claw.setPosition(clawOpen);
        sleep(500);
        elevatorGround();

        if (tagOfInterest == null || tagOfInterest.id == parkLeft) {
            //left code

        } else if (tagOfInterest.id == parkMiddle) {
            //middle code

        }
        else if (tagOfInterest.id == parkRight) {
            //right code

        }
    }
}

