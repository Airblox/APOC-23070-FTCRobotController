package com.example.meepmeepvisualiser;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualiser {
    public static void main(String[] args) {
        final double ONE_GRID = 70/3.0;
        final double TRACK_WIDTH = 16.75;

        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(ONE_GRID/2, -(70 - TRACK_WIDTH/2.0), Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .setStartPose(startPose)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .strafeToLinearHeading(new Vector2d(10, -ONE_GRID * 1.5), Math.PI)
                        .strafeToConstantHeading(new Vector2d(ONE_GRID * 2.1, -ONE_GRID * 1.2))
                        // First cycle
                        .setReversed(true)
                        .setTangent(Math.toRadians(152.5))
                        .splineToConstantHeading(new Vector2d(-ONE_GRID*2.5, -ONE_GRID/2), Math.toRadians(-180))
                        .setReversed(false)
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(ONE_GRID*2.1, -1.2*ONE_GRID), Math.toRadians(-27.5))
                        // Second cycle
                        .setReversed(true)
                        .setTangent(Math.toRadians(152.5))
                        .splineToConstantHeading(new Vector2d(-ONE_GRID*2.5, -ONE_GRID/2), Math.toRadians(-180))
                        .setReversed(false)
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(ONE_GRID*2.1, -1.2*ONE_GRID), Math.toRadians(-27.5))
                        // Parking
                        .strafeToConstantHeading(new Vector2d(ONE_GRID*2.1, -ONE_GRID/2))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}