package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="Parking", group="Parking")
public class Parking extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(35, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();

            TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(startPose)
                    .setTangent(-90)
                    .lineToLinearHeading(new Pose2d(35,24,Math.toRadians(90)))
                    .build();

            TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(startPose)
                    .setTangent(180)
                    .splineToSplineHeading(new Pose2d(22,65,Math.toRadians(90)),Math.toRadians(180))
                    .splineToConstantHeading( new Vector2d(9,35),Math.toRadians(-90))
                    .build();

            TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(startPose)
                    .setTangent(0)
                    .splineToConstantHeading(new Vector2d(65,55),Math.toRadians(-90))
                    .splineToConstantHeading( new Vector2d(65,25),Math.toRadians(-90))
                    .build();

                waitForStart();

            if (tagOfInterest == null || tagOfInterest.id == parkLeft)
            {
                //left code
                drive.followTrajectorySequence(ParkLeft);

            } else if (tagOfInterest.id == parkMiddle)
            {
                //middle code
                drive.followTrajectorySequence(ParkMid);

            } else if (tagOfInterest.id == parkRight)
            {
                //right code
                drive.followTrajectorySequence(ParkRight);

            }
        }
    }
}