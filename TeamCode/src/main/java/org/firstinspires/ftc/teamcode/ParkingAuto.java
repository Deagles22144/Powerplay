package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="ParkingAuto", group="AutoLeft")
public class ParkingAuto extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(36, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }




        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(startPose)
                .setTangent(-90)
                .lineToLinearHeading(new Pose2d(36,30,Math.toRadians(-90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(startPose)
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(36,48,Math.toRadians(-90)),Math.toRadians(-90))
                .splineToSplineHeading( new Pose2d(12,35,Math.toRadians(180)),Math.toRadians(180))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(36, 45, Math.toRadians(-90)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(65, 36,Math.toRadians(0)),Math.toRadians(10))
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