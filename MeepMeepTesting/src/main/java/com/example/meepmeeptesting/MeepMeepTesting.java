package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, 35, 0))
            .lineToLinearHeading(new Pose2d(-52, 48, 0))
            .waitSeconds(1)
            .lineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(-45)))
            .waitSeconds(1)
            .lineToLinearHeading(new Pose2d(-50, 59, Math.toRadians(0)))
            .waitSeconds(1)
            .lineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(-45)))
            .waitSeconds(1)
            .lineToLinearHeading(new Pose2d(-50, 59, Math.toRadians(20)))
            .waitSeconds(1)
            .lineToLinearHeading(new Pose2d(-56, 56, Math.toRadians(-45)))
            .build());

        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/into-the-deep-meepmeep-custom-field-images-printer-friendly-v0-qsax5fraignd1.png")); }
        catch(IOException ignored) {}

        assert img != null;
//        meepMeep.
        meepMeep.setBackground(img)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)

            .addEntity(myBot)
            .start();
    }
}