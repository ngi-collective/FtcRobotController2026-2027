/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

@TeleOp(name = "Concept: Vision Color-Locator (Circle)", group = "Concept")
public class VisionPollenTest extends LinearOpMode {
    @Override

    public void runOpMode() {
        //sets up the processor for the colors, const YELLOW meaning we are looking for the color of pollen (yellow)
        // YELLOW may not be the color we need tho so i (Hunter) NEED to run tests in order to figure out the color range we are looking for. 8/21/26
        // if you want to set up a new color range use this link: https://ftc-docs.firstinspires.org/en/latest/color_processing/color-locator-explore/color-locator-explore.html
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                /*    custom color scale in HSV test code. values must be tweaked. idk if this works though. - H
                .setTargetColorRange(new ColorRange(ColorSpace.HSV,
                        new Scalar(90, 50, 50),   // Lower bounds (e.g., Min H, Min S, Min V)
                        new Scalar(130, 255, 255))) // Upper bounds (e.g., Max H, Max S, Max V) )) */
                .setTargetColorRange(ColorRange.YELLOW) //another reminder that we may need to change this
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)// means only tracks outside border
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.9, 0.9, 0.9, -0.9)) //only looks at balls within a certain area
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 255)) // Draw a white circle
                .setBlurSize(5)

                // expands range then gets rid in order to add a buffer and fix holes
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        //init camera (init auramonster part happy birthday camera, i know what kind of aura you vibe with; the kind of aura of yellow balls)
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);// apparently this makes telemetry look different?

        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> pollens = colorLocator.getBlobs();

            //does the criteria so we can filter out small/big blobs and noncircular blobs
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    100, 20000, pollens);  // filter out very small blobs. the integer is area in pixels
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, pollens);

            telemetry.addData("Amount of Pollens:",pollens.size()); //number of pollens we detecting

            telemetry.addLine("Circularity Radius Center");

            // Display each Blob's circularity, and the size (radius) and center location of its circleFit
            for (ColorBlobLocatorProcessor.Blob b : pollens) {
                Circle circleFit = b.getCircle();
                telemetry.addLine(String.format("%5.3f      %3d     %3d",
                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX())); //notice only x value is mentioned, not y
            } /* hunter: this should give us where the pollen(s) is(are) in relation to our camera (just x value),
            letting us know which directions pollens are */

            telemetry.update();
            sleep(100); // Match the telemetry update interval.
        }
    }
}
