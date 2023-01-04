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
                .splineToSplineHeading(new Pose2d(36, 20, Math.toRadians(-90)), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    elevatorMid();
                })
                .splineToSplineHeading(new Pose2d(34, 14, Math.toRadians(-45)), Math.toRadians(180))
                /*.splineToSplineHeading(new Pose2d(51, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56, 15, Math.toRadians(-45)), Math.toRadians(45))
                */.build();

        TrajectorySequence coneLoad = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(-45))
                //.lineToLinearHeading((new Pose2d(58, 12, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(40, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)), Math.toRadians(0))
                //.setTangent(Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(clawOpen);
                })
                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .build();


        TrajectorySequence coneUnload = drive.trajectorySequenceBuilder(new Pose2d(63,12, Math.toRadians(0))/*coneLoad.end()*/)
                .setTangent(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    elevatorMid();
                })
                .splineToSplineHeading(new Pose2d(44,12,Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(34, 14, Math.toRadians(-45)), Math.toRadians(145))
               // .lineToLinearHeading(new Pose2d(58 , 15 ,Math.toRadians(-45)))
                .waitSeconds(1)
                /*.setTangent(45)
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-45)))*/
                .build();

        waitForStart();

        armPos(0.23);
        drive.followTrajectorySequence(Preload);
        sleep(1000);
        claw.setPosition(clawOpen);
        sleep(250);
        claw.setPosition(clawClose);
        sleep(100);
        elevatorGround();

        sleep(500);

        for (int i = 0; i < 5; i++){

            elevatorAuto(cones[i]);
            drive.followTrajectorySequence(coneLoad);
            claw.setPosition(clawClose);
            drive.setPoseEstimate(new Pose2d(63,drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            sleep(250);
            drive.followTrajectorySequence(coneUnload);
            sleep(1000);
            claw.setPosition(clawOpen);
            sleep(250);
            claw.setPosition(clawClose);
            sleep(100);
            elevatorGround();
        }

    }
}