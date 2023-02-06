
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="BlueLeft4Cones", group="AutoLeft")
public class BlueLeft4Cones extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(36, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }


        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(36 , 5, Math.toRadians(-90)), Math.toRadians(-90))
                /*.addDisplacementMarker(40, () -> {
                    armPos(0.6);
                })*/
                .addDisplacementMarker(() -> {
                    elevatorMid();
                    armPos(armMid);
//                    coneFliper.setPosition(coneFliperOpen);
                })
                .lineToSplineHeading(new Pose2d(30, 18 , Math.toRadians(-50)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence coneLoad = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(-50))
                //.lineToLinearHeading((new Pose2d(58, 12, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(40, 14, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(62, 14, Math.toRadians(0)), Math.toRadians(0))
                //.setTangent(Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(0)))
                // .addDisplacementMarker(() -> {
                //   claw.setPosition(clawOpen);
                //})
                .forward(4, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.05)
                .build();


        TrajectorySequence coneUnload = drive.trajectorySequenceBuilder(new Pose2d(63,13, Math.toRadians(0))/*coneLoad.end()*/)
                .setTangent(Math.toRadians(180))
                .addDisplacementMarker( () -> {
                    armPos(armMid-0.3);
                    elevatorMid();
                })
                .splineToSplineHeading(new Pose2d(45,14,Math.toRadians(0)), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> {
                    armPos(armMid);
                })
                .splineToSplineHeading(new Pose2d(30, 17, Math.toRadians(-50)), Math.toRadians(145))
                // .lineToLinearHeading(new Pose2d(58 , 15 ,Math.toRadians(-45)))
//                .waitSeconds(0.2)
                /*.setTangent(45)
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-45)))*/
                .build();

        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(0)
                .lineToLinearHeading(new Pose2d(38,20,Math.toRadians(-90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(210)
//                .splineToSplineHeading(new Pose2d(25,11,Math.toRadians(180)),Math.toRadians(180))
//                .splineToSplineHeading( new Pose2d(11,25,Math.toRadians(90)),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9,12,Math.toRadians(-90)))
                .back(5)
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(Math.toRadians(-50))
//                .splineToSplineHeading(new Pose2d(35, 25, Math.toRadians(0)),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(70, 35),Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(62,12,Math.toRadians(0)))

                .build();


        coneFliper.setPosition(coneFliperOpen);

        waitForStart();

        armPos(armLoadCone);
        sleep(300);
       // coneFliper.setPosition(coneFliperClose+0.2);
        drive.followTrajectorySequence(Preload);
        //armPos(armHigh);
        sleep(500);
        armPos(armPreRelease);
        claw.setPosition(clawOpen);
       // coneFliper.setPosition(coneFliperOpen);
        sleep(250);
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
            armPos(armMid);
            sleep(900);
            armPos(armPreRelease);
            claw.setPosition(clawOpen);
            sleep(500);
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