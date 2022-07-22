public class HelicopterController {
    private Point3D neededPosition;
    private double neededAngle;
    private PID pidPosition;
    private PID pidSupportAngle;
    private QuatenionPID forceAngle;
    private AircraftBase helicopter;

    public HelicopterController(Point3D neededPosition, double neededAngle, PID pidPosition, PID pidSupportAngle, QuatenionPID forceAngle, AircraftBase helicopter) {
        this.neededPosition = neededPosition;
        this.neededAngle = neededAngle;
        this.pidPosition = pidPosition;
        this.pidSupportAngle = pidSupportAngle;
        this.forceAngle = forceAngle;
        this.helicopter = helicopter;
    }

    public double mainRotorSpeed() {
        return pidPosition.calculateControl(-helicopter.getPosition().getY(),neededPosition.getY());
    }

    public double tailRotorSpeed() {
        return pidSupportAngle.calculateControl(Math.acos(helicopter.getAngle().getW())*2, Math.toRadians(neededAngle));
    }

    public Point3D getForceAngle() {
        double errX = neededPosition.getX()-helicopter.getPosition().getX();
        double errY = -(neededPosition.getY()-helicopter.getPosition().getY());
        double errZ = neededPosition.getZ()-helicopter.getPosition().getZ();

        Point3D errVec = Utils.rotY(-neededAngle).multiply(new Point3D(errX,errY,errZ));

        double angleX = Math.cos(Math.atan2(errVec.getY(),errVec.getX()));
        double angleZ = Math.cos(Math.atan2(errVec.getY(),errVec.getZ()));

        //to do пид для кватернонов.
        Quaternion q = new Quaternion(Utils.eulerAnglesToQuaternion(new Point3D(-angleZ,0,angleX))).multiply(helicopter.getAngle().conjugate());
        return Utils.quaternionRotation(q).multiply(new Point3D(0,1,0));

    }
}
