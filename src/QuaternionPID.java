public class QuaternionPID {
    private final SimulationEnvironment simulationEnvironment;

    private Point3D lastState;
    private Point3D sum;

    private final double p, i, d;


    public QuaternionPID(double p, double i, double d, SimulationEnvironment simulationEnvironment) {
        this.simulationEnvironment = simulationEnvironment;
        lastState = new Point3D();
        sum = new Point3D();
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public Point3D calculateControl(Quaternion current, Quaternion destination) {
        Quaternion error = destination.multiply(current.conjugate());
        Point3D vecError = new Point3D(error.getX(),error.getY(),error.getZ());
        sum = sum.add(vecError).multiply(simulationEnvironment.getDeltaTime());

        Point3D alpha = vecError.multiply(p);
        Point3D betta = sum.multiply(p);
        Point3D gamma = (vecError.subtract(vecError)).multiply(d/simulationEnvironment.getDeltaTime());

        lastState = vecError;

        return alpha.add(betta).add(gamma);
    }
}
