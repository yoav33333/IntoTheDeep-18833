package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;


import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
public class java5spec extends AutoBaseJava {
    public java5spec() {
        super(Side.chamber);
    }

    static Pose startPose = startingPoseChamber;
    static Pose chamberPose1 = new Pose(-35, 8, Math.toRadians(180));
    static Pose chamberPose2 = new Pose(-33.5, 4, Math.toRadians(180));
    static Pose chamberPose3 = new Pose(-35, 0, Math.toRadians(180));
    static Pose chamberPose4 = new Pose(-35, -4, Math.toRadians(180));
    static Pose chamberPose5 = new Pose(-35, -8, Math.toRadians(180));
    static Pose dragPose1 = new Pose(-40, -23, Math.toRadians(-45));
    static Pose dragPose2 = new Pose(-40, -34, Math.toRadians(-45));
    static Pose dragPose3 = new Pose(-40, -45, Math.toRadians(-45));
    static Pose dragPoseTurn1 = new Pose(-40, -23, Math.toRadians(-45-90));
    static Pose dragPoseTurn2 = new Pose(-40, -34, Math.toRadians(-45-90));
    static Pose dragPoseTurn3 = new Pose(-40, -45, Math.toRadians(-45-90));
    static Pose pickup1Pose = new Pose(-57.5, -40, Math.toRadians(0));
    static Pose pickup2Pose = new Pose(-43, -15, Math.toRadians(180+50));

    static PathChain scorePreload;
    static PathChain getToDrag1;
    static PathChain turnDrag1;
    static PathChain getToDrag2;
    static PathChain turnDrag2;
    static PathChain getToDrag3;
    static PathChain turnDrag3;
    static PathChain specialHPIntake;
    static PathChain dragToHP;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
    static PathChain scorePickup4;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
    static PathChain pickup4;
    static PathChain park;
    @Override
    public void myInit() {
        scorePreload = makeLinePath(startPose, chamberPose1);
        getToDrag1 = makeLinePath(chamberPose1, dragPose1);
        turnDrag1 = makeLinePath(dragPose1, dragPoseTurn1);
        getToDrag2 = makeLinePath(dragPoseTurn1, dragPose2);
        turnDrag2 = makeLinePath(dragPose2, dragPoseTurn2);
        getToDrag3 = makeLinePath(dragPoseTurn2, dragPose3);
        turnDrag3 = makeLinePath(dragPose3, dragPoseTurn3);
        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, chamberPose2);
        scorePickup2 = makeLinePath(pickup2Pose, chamberPose3);
        scorePickup3 = makeLinePath(pickup2Pose, chamberPose4);
        scorePickup4 = makeLinePath(pickup2Pose, chamberPose5);
        pickup1 = makeLinePath(chamberPose2, pickup2Pose);
        pickup2 = makeLinePath(chamberPose3, pickup2Pose);
        pickup3 = makeLinePath(chamberPose4, pickup2Pose);
        pickup4 = makeLinePath(chamberPose5, pickup2Pose);
//        park = makeLinePath(basketScore, parkPose);
        clawSubsystem.getClawRotationServo().setPosition(0.5);
        deposit.closeClaw();
        deposit.armOutHalf();
    }

    @Override
    public void myStart() {
        deposit.armOut();
        linearSlides.getRunToPosition().schedule();
        new Sequential(
            extendoCommand.getExtendoReset(),
            linearSlides.getGoToHighChamber(),
            followPath(scorePreload),
            deposit.getSlamSeq(),
            new Parallel(
                new Sequential(
                    new Wait(0.4),
                    linearSlides.getGoToLowChamber()
                ),
                followPath(getToDrag1)
            ),
            turnTo(-45-90).with(new Wait(1)),
            followPath(getToDrag2),
            turnTo(-45-90).with(new Wait(1)),
            followPath(getToDrag3),
            turnTo(-45-90).with(new Wait(1)),

            new Parallel(
            followPath(specialHPIntake),
            new Sequential(
                deposit.getIntakeCommand(),
                deposit.getRelease(),
                new Wait(0.5),
                getCatchPixel(),
                new Wait(0.3),
                getPostIntakeState()
            )
            ),
            linearSlides.getGoToHighChamber(),
            followPath(scorePickup1),
            deposit.getSlamSeq(),
            new Parallel(
                new Sequential(
                    new Wait(0.4),
                    linearSlides.getGoToLowChamber()
                ),
                followPath(pickup1)
            ),
//            followPath(pickup1),
            followPath(scorePickup2),
            followPath(pickup2),
            followPath(scorePickup3),
            followPath(pickup3),
            followPath(scorePickup4),
            followPath(pickup4)
        ).schedule();
    }
}
