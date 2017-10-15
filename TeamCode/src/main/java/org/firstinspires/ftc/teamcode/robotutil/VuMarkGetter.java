package org.firstinspires.ftc.teamcode.robotutil;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;


public class VuMarkGetter {
    public VuforiaTrackables getVuforia() {
        return getVuforia(true);
    }

    public VuforiaTrackables getVuforia(boolean showCameraView) {

        VuforiaLocalizer.Parameters parameters;
        if (showCameraView) {

            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
            // Makes the Vuforia view appear on the phone screen
            // Can remove the R.id.cameraMonitorViewId to save battery or whatever.
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AQgn1d//////AAAAGS+F+GWwAEbtqn64lm+fvolRqft5tIJLGdUCsB51qVZHMP3UU8cTCBMKvjCBUTxHfkooO1dljaRLNzaDMMTbWw978Agd7qMrUQF/I4dsE+oVUhLVTHxHPl4r8T4LJ1+B5KHvXQyTr7S3bTU1xy/id/uACCppztVO6mH6Aj0FwY/v3lDYnL9sQNVi2DNXNrnQmmshyJC74C4Se8a6A/II7vcaQ00Ot3PlSB9LjH6K28EQ3oiLnc6tKTGjbU+uTBdoix2KUDL7xVa8c6biG2lcuu7j6dRrw/uvUrh7RpWcmvQDdoshtLlXLsvacLwr5NzMX+4quVkydj/3KRrixOKnepk0ZSPiSlt+J+ThynHcgevu";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        // Can also use a teapot.

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        // We only need one vision target for this year.

        VuforiaTrackables trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        trackables.setName("Vision Targets");
        return trackables;
    }

    public RelicRecoveryVuMark getPattern(VuforiaTrackables vuforiaTrackables){
        VuforiaTrackable target = vuforiaTrackables.get(0);
        return RelicRecoveryVuMark.from(target);
    }

    public DistanceOffsets getOffset(VuforiaTrackables vuforiaTrackables) {
        VuforiaTrackable target = vuforiaTrackables.get(0);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
        RelicRecoveryVuMark type = RelicRecoveryVuMark.from(target);
        if (pose != null) {
            double distanceLeftRight = pose.getTranslation().get(0);
            double distanceUpDown = pose.getTranslation().get(1);
            double distanceForwardsBackwards = Math.abs(pose.getTranslation().get(2));
            return new DistanceOffsets(distanceForwardsBackwards, distanceLeftRight, distanceUpDown, type);
        }
        return null;
        //Found nothing
    }

    public class DistanceOffsets {
        public double distance, horizontal, vertical;
        public boolean foundValues;
        public RelicRecoveryVuMark vuMarkType;

        DistanceOffsets(double forwardsBack, double leftRight, double upDown, RelicRecoveryVuMark type) {
            foundValues = true;
            distance = forwardsBack;
            horizontal = leftRight;
            vertical = upDown;
            vuMarkType = type;
        }

        DistanceOffsets() {
            this.foundValues = false;
            distance = horizontal = vertical = 0;
            vuMarkType = RelicRecoveryVuMark.UNKNOWN;
        }
    }
}