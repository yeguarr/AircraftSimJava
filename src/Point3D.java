public class Point3D {
    private final Matrix.m4x1 coordinate;

    Point3D() {
        coordinate = new Matrix.m4x1(0,0,0,1);
    }
    Point3D(double x, double y, double z) {
        coordinate = new Matrix.m4x1(x,y,z,1);
    }
    Point3D(double x, double y, double z,double w) {
        coordinate = new Matrix.m4x1(x,y,z,w);
    }
    Point3D(Point3D point3D) {
        coordinate = new Matrix.m4x1(point3D.coordinate.getA(),point3D.coordinate.getB(),point3D.coordinate.getC(),point3D.coordinate.getD());
    }
    Point3D(Matrix.m4x1 coordinate) {
        this.coordinate = new Matrix.m4x1(coordinate.getA(),coordinate.getB(),coordinate.getC(),coordinate.getD());
    }

    public Matrix.m4x1 getCoordinate() {
        return coordinate;
    }

    public double getX() {
        return coordinate.getA() / coordinate.getD();
    }

    public double getY() {
        return coordinate.getB() / coordinate.getD();
    }

    public double getZ() {
        return coordinate.getC() / coordinate.getD();
    }

    public Point3D add(Point3D point) {
        return add(point.getX(),point.getY(),point.getZ());
    }

    public Point3D add(double x, double y, double z) {
        double xp = coordinate.getD() * (getX() + x);
        double yp = coordinate.getD() * (getY() + y);
        double zp = coordinate.getD() * (getZ() + z);
        return new Point3D(xp,yp,zp,coordinate.getD());
    }

    public Point3D subtract(Point3D point) {
        return subtract(point.getX(),point.getY(),point.getZ());
    }

    public Point3D subtract(double x, double y, double z) {
        double xp = coordinate.getD() * (getX() - x);
        double yp = coordinate.getD() * (getY() - y);
        double zp = coordinate.getD() * (getZ() - z);
        return new Point3D(xp,yp,zp,coordinate.getD());
    }

    public Point3D multiply(double k) {
        return new Point3D(this.getX()*k,this.getY()*k,this.getZ()*k);
    }
    public Point3D scale(double x, double y, double z) {
        return new Point3D(this.getX()*x,this.getY()*y,this.getZ()*z);
    }

    public double dot(Point3D second) {
        return this.getX() * second.getX() + this.getY() * second.getY() + this.getZ() * second.getZ();
    }
    public Point3D cross(Point3D second) {
        return new Point3D(this.getY()*second.getZ()-this.getZ()*second.getY(),
                this.getZ()*second.getX()-this.getX()*second.getZ(),
                this.getX()*second.getY()-this.getY()*second.getX());
    }

    public Point3D normalise() {
        double r = length();
        return new Point3D(getX()/r,getY()/r,getZ()/r);
    }

    public double length() {
        return Math.sqrt(this.dot(this));
    }

    @Override
    public String toString() {
        return coordinate.getA()+" "+coordinate.getB()+" "+coordinate.getC()+" "+coordinate.getD();
    }

}
