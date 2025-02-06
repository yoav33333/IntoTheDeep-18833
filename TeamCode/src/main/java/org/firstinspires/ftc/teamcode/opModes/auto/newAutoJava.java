package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.commands.extendoCommand;


import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
@Autonomous
public class newAutoJava extends AutoBaseJava {
    public newAutoJava() {
        super(AutoBaseJava.Side.basket);
    }

    static Pose startPose = startingPoseBasket;
    static Pose chamberPose = new Pose(36.04117647058824, 82.412, Math.toRadians(180.0));
    static Pose pickup1Pose = new Pose(19.28823529411765, 118.70588235294117, Math.toRadians(0.0));
    static Pose basketScore = new Pose(16.059, 126.382, Math.toRadians(310.0));
    static Pose pickup2Pose = new Pose(20.737, 129.382, Math.toRadians(0.0));
    static Pose parkPose = new Pose(59.82352941176471, 102.59823529, 0.0);

    static PathChain scorePreload;
    static PathChain grabPickup1;
    static PathChain scorePickup1;
    static PathChain park;
    @Override
    public void myInit() {
        scorePreload = makeLinePath(startPose, chamberPose);
        grabPickup1 = makeLinePath(chamberPose, pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, basketScore);
        park = makeLinePath(basketScore, parkPose);
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
                new Wait(0.2),
                deposit.getSlamSeq(),
                new Wait(0.2),
                new Parallel(
                        followPath(grabPickup1),
                        new Sequential(
                                new Wait(0.5),
                                getExtendoOpenCommand()
                        )
                ),
                followPath(grabPickup1),
                new Wait(1.0),
                clawSubsystem.getCloseClaw(),
                new Wait(1.2),
                new Parallel(
                        extendoCommand.getExtendoCloseCommandAuto(),
                        linearSlides.getGoToHighBasket()
                        ),
                new Wait(2.0),
                followPath(scorePickup1),
                new Wait(0.1),
                deposit.getRelease(),
                new Wait(0.2),
                followPath(park),
                linearSlides.getGoToLowChamber()
        ).schedule();
    }
}
