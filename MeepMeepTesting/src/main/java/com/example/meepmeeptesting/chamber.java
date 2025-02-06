package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class chamber {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -10, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-35, 8, Math.toRadians(180)))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-46, -23, Math.toRadians(-45)))
                .waitSeconds(0)
                .turn(Math.toRadians(-90))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-46, -34, Math.toRadians(-45)))
                .waitSeconds(0)
                .addTemporalMarker(0.0, ()->System.out.println("hi"))
                .turn(Math.toRadians(-90))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-46, -45, Math.toRadians(-45)))
                .waitSeconds(0)
                .addTemporalMarker(0.0, ()->System.out.println("hi"))
                .turn(Math.toRadians(-90))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(0)))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-35, 4, Math.toRadians(180)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(0)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-35, 0, Math.toRadians(180)))
                .waitSeconds(0)
                .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(0)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-35, -4, Math.toRadians(180)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(0)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-35, -8, Math.toRadians(180)))
                .waitSeconds(0.0)
                .lineToLinearHeading(new Pose2d(-60, -42, Math.toRadians(0)))
                .build());

        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/into-the-deep-meepmeep-custom-field-images-printer-friendly-v0-qsax5fraignd1.png")); }
        catch(IOException ignored) {}

        assert img != null;
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(myBot)
                .start();
    }
}
