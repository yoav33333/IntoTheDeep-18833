package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getArmIn;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;
import static org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem;

import java.util.function.BooleanSupplier;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
@Config
@Autonomous
public class java4sample extends AutoBaseJava {
    public java4sample() {super(Side.basket);}

    public static Pose startPose = startingPoseBasket;
    public static Pose basketPose1 = new Pose(-58, 55, Math.toRadians(-45));
    public static Pose basketPose2 = new Pose(-56.5, 56, Math.toRadians(-45));
    public static Pose basketPose3 = new Pose(-56.5, 57, Math.toRadians(-45));
    public static Pose basketPose4 = new Pose(-55, 58, Math.toRadians(-45));
    public static Pose pickup1Pose = new Pose(-53.5, 48.1, 0);
    public static Pose pickup2Pose = new Pose(-54.5, 56.7, Math.toRadians(0));
    public static Pose pickup3Pose = new Pose(-53, 60, Math.toRadians(21));
    public static Pose parkPose = new Pose(-7, 19, Math.toRadians(90));
    public static Pose parkControl = new Pose(-9, 49, Math.toRadians(0));

    static PathChain scorePreload;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
    static PathChain park;
    @Override
    public void myInit() {

        scorePreload = makeLinePath(startPose, basketPose1);
        pickup1 = makeLinePath(basketPose1, pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, basketPose2);
        pickup2 = makeLinePath(basketPose2, pickup2Pose);
        scorePickup2 = makeLinePath(pickup2Pose, basketPose3);
        pickup3 = makeLinePath(basketPose3, pickup3Pose);
        scorePickup3 = makeLinePath(pickup3Pose, basketPose4);
        park = makeCurvePath(basketPose4, parkControl, parkPose);

        clawSubsystem.getClawRotationServo().setPosition(0.5);
        clawSubsystem.getClawServo().setPosition(0.0);
        deposit.armOutHalf();
        deposit.closeClawRaw();

    }


    @Override
    public void myStart() {

        linearSlides.setPose(0);
        linearSlides.getRunToPosition().cancel();
        linearSlides.getRunToPosition().schedule();
        new Sequential(
            extendoCommand.getExtendoReset(),
            linearSlides.getGoToHighBasket(),
            runNonBlocking(
                    new Sequential(
                        waitUntil(()->(linearSlides.target>500 && linearSlides.target-300<getPose())) ,
                        deposit.getArmOut()
                    )),
            waitUntil(() -> getPose()>2000),
            followPath(scorePreload),
            waitUntil(() -> getPose()>3400),
            deposit.getRelease(),
            new Wait(0.1),
            new Parallel(
                new Sequential(
                    waitUntil(()->follower.getCurrentTValue()>0.1),
                    extendoCommand.getExtendoOpenCommandAuto()
                ),
                followPath(pickup1)
            ),
            new Wait(0.2),
            clawSubsystem.getCloseClaw(),
            new Wait(0.2),
            linearSlides.getGoToHighBasket(),
            extendoCommand.getExtendoCloseCommandAuto(),
            waitUntil(() -> getPose()>2200),
            followPath(scorePickup1),
            waitUntil(() -> getPose()>3400),
            deposit.getRelease(),
            new Wait(0.1),
            new Parallel(
                extendoCommand.getExtendoOpenCommandAuto(),
                followPath(pickup2)
            ),
            new Wait(0.3),
            clawSubsystem.getCloseClaw(),
            new Wait(0.2),
            linearSlides.getGoToHighBasket(),
            extendoCommand.getExtendoCloseCommandAuto(),
            waitUntil(() -> getPose()>2200),
            followPath(scorePickup2),
            waitUntil(() -> getPose()>3400),
            deposit.getRelease(),
            new Wait(0.1),
            new Parallel(
                extendoCommand.getExtendoOpenCommandAuto(),
                followPath(pickup3)
            ),
            new Wait(0.3),
            clawSubsystem.getCloseClaw(),
            new Wait(0.2),
            linearSlides.getGoToHighBasket(),
            extendoCommand.getExtendoCloseCommandAuto(),
            waitUntil(() -> getPose()>2200),
            followPath(scorePickup3),
            waitUntil(() -> getPose()>3400),
            deposit.getRelease(),
            new Wait(0.1),
            new Parallel(
                followPath(park),
                new Sequential(
                    waitUntil(()->follower.getCurrentTValue()>0.3),
                    linearSlides.getTouchBar(),
                    getArmIn()
                )
            ),

            finishAuto
        ).schedule();
    }
    @Override
    public void myStop() {
        follower.breakFollowing();
        runFollower.cancel();
        linearSlides.getRunToPosition().cancel();
        linearSlides.setStartingPose(getPose());
//        followerSubsystem.setStartingPose(follower.getPose());
    }
}
