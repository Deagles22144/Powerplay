package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="BlueLeftLow", group="AutoLeft")
public class BlueLeftLow extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(36, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }

        /*TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(35, 17, Math.toRadians(-90)), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    elevatorLow();
                })
                .splineToSplineHeading(new Pose2d(42, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(51, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56, 15, Math.toRadians(-45)), Math.toRadians(45))
                .build();

        TrajectorySequence coneLoad = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(180))
                .lineToLinearHeading((new Pose2d(58, 12, Math.toRadians(0))))
                *//*.splineToSplineHeading(new Pose2d(58, 15, Math.toRadians(-45)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(0)))
                *//*
                .waitSeconds(1)
                .build();

        TrajectorySequence coneUnload = drive.trajectorySequenceBuilder(coneLoad.end())
                .setTangent(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    elevatorLow();
                })
                .lineToLinearHeading(new Pose2d(58 , 15 ,Math.toRadians(-45)))
                .waitSeconds(1)

                *//*.setTangent(45)
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-45)))*//*
                .build();*/

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(36 , 20, Math.toRadians(-90)), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    elevatorMid();
                    armPos(armMid);
                })
                .lineToSplineHeading(new Pose2d(31, 18, Math.toRadians(-50)))
                .build();

        TrajectorySequence coneLoad = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(-45))
                //.lineToLinearHeading((new Pose2d(58, 12, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(40, 13, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(63, 13, Math.toRadians(0)), Math.toRadians(0))
                //.setTangent(Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(0)))
               // .addDisplacementMarker(() -> {
                 //   claw.setPosition(clawOpen);
                //})
                .forward(4.25, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.17)
                .build();


        TrajectorySequence coneUnload = drive.trajectorySequenceBuilder(new Pose2d(63,13, Math.toRadians(0))/*coneLoad.end()*/)
                .setTangent(Math.toRadians(180))
                .addDisplacementMarker( () -> {
                    elevatorMid();
                    armPos(armMid);
                })
                .splineToSplineHeading(new Pose2d(44,12,Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(30, 17, Math.toRadians(-50)), Math.toRadians(145))
               // .lineToLinearHeading(new Pose2d(58 , 15 ,Math.toRadians(-45)))
                .waitSeconds(0.2)
                /*.setTangent(45)
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-45)))*/
                .build();

        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(0)
                .lineToLinearHeading(new Pose2d(35,24,Math.toRadians(-90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(210)
                .splineToSplineHeading(new Pose2d(28,12,Math.toRadians(0)),Math.toRadians(180))
                .splineToSplineHeading( new Pose2d(11,25,Math.toRadians(-90)),Math.toRadians(90))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(63, 38, Math.toRadians(180)))
                .build();


        waitForStart();

        armPos(0.6);
        drive.followTrajectorySequence(Preload);
        //armPos(armHigh);
        sleep(850);
        armPos(armPreRelease);
        claw.setPosition(clawOpen);
        sleep(275);
        claw.setPosition(clawClose);
        sleep(200);
        elevatorGround();
        sleep(300);

        for (int i = 0; i < 3; i++) {

            elevatorAuto(cones[i]);
            claw.setPosition(clawOpen);
            drive.followTrajectorySequence(coneLoad);
            claw.setPosition(clawClose);
            drive.setPoseEstimate(new Pose2d(63, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            sleep(200);
            elevatorTargetPosition(cones[i] + 350);
            sleep(200);
            drive.followTrajectorySequence(coneUnload);
           // armPos(armHigh);
            //sleep(100);
            armPos(armPreRelease);
            claw.setPosition(clawOpen);
            sleep(200);
            claw.setPosition(clawClose);
            sleep(200);
            elevatorGround();
            sleep(300);
        }

            if (tagOfInterest == null || tagOfInterest.id == parkLeft)
            {
                //left code
                armPos(armGround);
                drive.followTrajectorySequence(ParkLeft);

            } else if (tagOfInterest.id == parkMiddle)
            {
                //middle code
                armPos(armGround);
                drive.followTrajectorySequence(ParkMid);

            } else if (tagOfInterest.id == parkRight)
            {
                //right code
                armPos(armGround);
                drive.followTrajectorySequence(ParkRight);

            }





    }
}