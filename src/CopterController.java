public class CopterController {
    QuadCopter copter;
    double neededThrust;
    double neededYaw;
    double neededPitch;
    double neededRoll;

    Quaternion referenceRotation;
    Quaternion errorRotation;

    void mixMotors() {
        copter.getFirstPropeller().setSpeed(neededThrust+neededYaw+neededPitch-neededRoll);
        copter.getFirstPropeller().setSpeed(neededThrust-neededYaw+neededPitch-neededRoll);
        copter.getFirstPropeller().setSpeed(neededThrust-neededYaw-neededPitch+neededRoll);
        copter.getFirstPropeller().setSpeed(neededThrust+neededYaw+neededPitch+neededRoll);
    }

    void calculateRotation(Quaternion currentRotation) {
        errorRotation = referenceRotation.multiply(currentRotation.conjugate());
    }
}
