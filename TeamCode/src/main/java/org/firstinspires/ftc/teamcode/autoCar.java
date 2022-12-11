package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="autoCar", group="Robot")
public class autoCar extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();

        }

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 25), Math.toRadians(-90))
                .addDisplacementMarker(5,() -> {
                    elevatorHigh();
                    tilt.setPosition(tiltAuto);
                })
                .splineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(135)), Math.toRadians(-70))
                .waitSeconds(1)
                .build();


        TrajectorySequence FirstDriveToCones = drive.trajectorySequenceBuilder(Preload.end())//FirstBackDrive.end())
                .setTangent(Math.toRadians(120))
                .splineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,15, Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(5,() -> {
                    elevatorAuto(cones[0]);
                    //tilt.setPosition(tiltAuto);
                })
                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



        TrajectorySequence ConeUnload = drive.trajectorySequenceBuilder(new Pose2d(-63,15, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-45, 15, Math.toRadians(180)),Math.toRadians(0))
                .addDisplacementMarker(2,() -> {
                    elevatorHigh();
                    //tilt.setPosition(tiltAuto);
                 })
                .splineToSplineHeading(new Pose2d(-32, 10 , Math.toRadians(135)),Math.toRadians(-50))
                .waitSeconds(1)
                .build();

        TrajectorySequence ConeLoad = drive.trajectorySequenceBuilder(ConeUnload.end())//FirstBackDrive.end())
                .setTangent(Math.toRadians(120))
                .splineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,15, Math.toRadians(180)),Math.toRadians(180))
//                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence forwardDrive = drive.trajectorySequenceBuilder(ConeLoad.end())
                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(120)
                .lineToLinearHeading(new Pose2d(-35,24,Math.toRadians(90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-35,25,Math.toRadians(115)),Math.toRadians(115))
                .splineToSplineHeading( new Pose2d(-64,35,Math.toRadians(180)),Math.toRadians(180))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-25,12,Math.toRadians(215)),Math.toRadians(45))
                .splineToSplineHeading( new Pose2d(-13,25,Math.toRadians(-90)),Math.toRadians(90))
                .build();


        waitForStart();

       // armPos(0.04);

        drive.followTrajectorySequence(Preload);

        if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20) || elevator0.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
        {
            claw.setPosition(clawOpen);
            sleep(750);
            elevatorGround();
        }

        while (elevator0.getCurrentPosition() >= elevatorMiddlePos / 2)
        {
            armPos(0.05);
        }

        armPos(0.01);
        tilt.setPosition(tiltGround);
        sleep(600);

        drive.followTrajectorySequence(FirstDriveToCones);

        for (int i = 1; i < 3; i++)
        {

            drive.setPoseEstimate(new Pose2d(-63,15, Math.toRadians(180)));

            claw.setPosition(clawClose);
            tilt.setPosition(tiltLow);
            sleep(1000);
            elevatorAfterColloctAuto();

            drive.followTrajectorySequence(ConeUnload);
/*
            elevatorHigh();
            tilt.setPosition(tiltAuto);

            sleep(2500);
            */

           // drive.followTrajectorySequence(SecondBackDrive);
            //drive.followTrajectorySequence(SecondBackDrive);
            telemetry.addLine("elevator pos: " + elevator1.getCurrentPosition());
            if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
            {
                claw.setPosition(clawOpen);
                sleep(750);
            }

            //elevatorAuto(cones[i]);
            elevatorGround();
            while (elevator0.getCurrentPosition() >= 300)
            {
                armPos(0.05);
            }

            armPos(0.01);
            tilt.setPosition(tiltGround);
            sleep(600);

            if(i < 2)
            {
                drive.followTrajectorySequence(ConeLoad);
                elevatorAuto(cones[i]);
                drive.followTrajectorySequence(forwardDrive);
            }
        }

        elevatorGround();

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



