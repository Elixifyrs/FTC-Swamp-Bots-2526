package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;


@Config
@Autonomous(name = "LearningDemo", group = "RR1.0 Auto")
public class LearningDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //We start at 26, -55.5, heading 90
        Pose2d beginPose = new Pose2d(8, -62, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Build Trajectories

        //Wait for arm and move to bar
        Action RightofPole6 = drive.actionBuilder(beginPose)
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(40,-38), Math.toRadians(90))
                .build();

        //Move back from bar and prime sample 1
//        Action CrossPole6 = drive.actionBuilder(beginPose)
//                //.splineToLinearHeading(new Pose2d(12,-12, Math.toRadians(180)), Math.toRadians(90))
//                .build();
//
//        //Retrieve sample 1 and prime sample 2
//        Action primeSample2 = drive.actionBuilder(beginPose)
//                .lineToYConstantHeading(-42)
//                .waitSeconds(.1)
//                .lineToYConstantHeading(-8)
//                .splineToConstantHeading(new Vector2d(60,-4),Math.toRadians(90),
//                        new TranslationalVelConstraint(15))
//                .build();
//
//        //Retrieve sample 2
//        Action retrieveSample2 = drive.actionBuilder(beginPose)
//                .lineToY(-48)
//                .build();
//
//        //Align Specimen 1
//        Action alignSpecimen1 = drive.actionBuilder(beginPose)
//                .splineToLinearHeading(new Pose2d(50,-46, Math.toRadians(270)), Math.toRadians(90))
//                .waitSeconds(.1)
//                .build();
//
//        //Take Specimen 1
//        Action takeSpecimen1 = drive.actionBuilder(beginPose)
//                .lineToYConstantHeading(-54.5, new TranslationalVelConstraint(15))
//                .waitSeconds(.250)
//                .build();
//
//        //Move to the bar
//        Action moveToBar1 = drive.actionBuilder(beginPose)
//                .waitSeconds(250)
//                .lineToYConstantHeading(-54.5, new TranslationalVelConstraint(15))
//                .splineToLinearHeading(new Pose2d(3,-30, Math.toRadians(90)), Math.toRadians(270))
//                .waitSeconds(.1)
//                .lineToYConstantHeading(-26)
//                .build();

        waitForStart();

        if (isStopRequested()) return;

        //Close hand to start
        Actions.runBlocking(
                new SequentialAction(
                        RightofPole6
                        //CrossPole6
                )
        );
    }
}
