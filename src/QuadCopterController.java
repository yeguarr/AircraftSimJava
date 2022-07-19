public class QuadCopterController {
    AircraftBase copter;

    AircraftBase.Propeller m1;
    AircraftBase.Propeller m2;
    AircraftBase.Propeller m3;
    AircraftBase.Propeller m4;

    Point3D currentPosition;
    Point3D neededPosition;

    Quaternion referenceRotation;
    Quaternion errorRotation;

    void calculateMotorSpeed() {

    }

    void calculateRotation(Quaternion currentRotation) {
        errorRotation = referenceRotation.multiply(currentRotation.conjugate());
    }
}
