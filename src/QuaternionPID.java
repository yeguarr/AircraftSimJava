public class QuaternionPID {
    private final SimulationEnvironment simulationEnvironment;

    private Quaternion lastState;
    private Quaternion sum;

    private final double p, i, d;

    public QuaternionPID(double p, double i, double d, SimulationEnvironment simulationEnvironment) {
        lastState = new Quaternion(1,0,0,0);
        sum = new Quaternion(1,0,0,0);
        this.p = p;
        this.i = i;
        this.d = d;
        this.simulationEnvironment = simulationEnvironment;
    }

    public Quaternion calculateControl(Quaternion current, Quaternion destination) {
        Quaternion error = current.multiply(destination.conjugate());
        sum = sum.multiply(error.multiply(simulationEnvironment.getDeltaTime()));
        Quaternion alpha = error.multiply(p);
        Quaternion betta = sum.multiply(i);
        Quaternion gamma = (error.multiply(lastState.conjugate())).multiply(d/simulationEnvironment.getDeltaTime());
        lastState =  new Quaternion(error);

        return alpha;//.multiply(betta).multiply(gamma);
    }
}
