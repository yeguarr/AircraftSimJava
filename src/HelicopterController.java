public class HelicopterController {
    private Point3D neededPosition;
    private double neededAngle;
    private PID pidPosition;
    private PID pidSupportAngle;
    private QuaternionPID forceAngle;

    public HelicopterController(Point3D neededPosition, double neededAngle, PID pidPosition, PID pidSupportAngle, QuaternionPID forceAngle) {
        this.neededPosition = neededPosition;
        this.neededAngle = neededAngle;
        this.pidPosition = pidPosition;
        this.pidSupportAngle = pidSupportAngle;
        this.forceAngle = forceAngle;
    }

    public double mainRotorSpeed(Point3D helicopterPosition) {
        return pidPosition.calculateControl(-helicopterPosition.getY(),neededPosition.getY());
    }

    public double tailRotorSpeed(Quaternion helicopterAngle) {
        return pidSupportAngle.calculateControl(Math.acos(helicopterAngle.getW())*2, Math.toRadians(neededAngle));
    }

    public Point3D getForceAngle(Point3D helicopterPosition,Quaternion helicopterAngle) {
        double errX = neededPosition.getX()-helicopterPosition.getX();
        double errY = -(neededPosition.getY()-helicopterPosition.getY());
        double errZ = neededPosition.getZ()-helicopterPosition.getZ();

        Point3D errVec = Utils.rotY(-neededAngle).multiply(new Point3D(errX,errY,errZ));

        double angleX = Math.cos(Math.atan2(errVec.getY(),errVec.getX()));
        double angleZ = Math.cos(Math.atan2(errVec.getY(),errVec.getZ()));

        //to do пид для кватернонов.
        Quaternion q = forceAngle.calculateControl(Utils.eulerAnglesToQuaternion(new Point3D(-angleZ,0,angleX)),helicopterAngle);
        //Quaternion q = new Quaternion(Utils.eulerAnglesToQuaternion(new Point3D(-angleZ,0,angleX))).multiply(helicopter.getAngle().conjugate());

        return Utils.quaternionRotation(q).multiply(new Point3D(0,1,0));

    }
}
