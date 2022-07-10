public class Quaternion {
    private final Matrix.m4x1 coordinate;
    Quaternion() {
        coordinate = new Matrix.m4x1(0,0,0,0);
    }
    Quaternion(double w, double x, double y, double z) {
        coordinate = new Matrix.m4x1(w,x,y,z);
    }
    Quaternion(double w, Point3D point3D) {
        coordinate = new Matrix.m4x1(w,point3D.getX(),point3D.getY(),point3D.getZ());
    }
    Quaternion(Matrix.m4x1 coordinate) {
        this.coordinate = new Matrix.m4x1(coordinate.getA(),coordinate.getB(),coordinate.getC(),coordinate.getD());
    }
    Quaternion(Quaternion quaternion) {
        this.coordinate = new Matrix.m4x1(quaternion.coordinate.getA(),quaternion.coordinate.getB(),quaternion.coordinate.getC(),quaternion.coordinate.getD());
    }

    public Matrix.m4x1 getCoordinate() {
        return coordinate;
    }

    public double getW() {
        return coordinate.getA() ;
    }

    public double getX() {
        return coordinate.getB() ;
    }

    public double getY() {
        return coordinate.getC() ;
    }

    public double getZ() {
        return coordinate.getD() ;
    }

    public Point3D getVector() {
        return new Point3D(getX(),getY(),getZ());
    }

    public Quaternion add(Quaternion point) {
        return new Quaternion(this.getW() + point.getW(),this.getX() + point.getX(),this.getY() + point.getY(),this.getZ()+  point.getZ());
    }

    public Quaternion subtract(Quaternion point) {
        return new Quaternion(this.getW() - point.getW(), this.getX() - point.getX(), this.getY() - point.getY(), this.getZ() - point.getZ());
    }

    public Quaternion multiply(Quaternion quaternion) {
        Point3D v1 = this.getVector();
        Point3D v2 = quaternion.getVector();
        double w1 = this.getW();
        double w2 = quaternion.getW();
        return new Quaternion(w1*w2-v1.dot(v2),v2.multiply(w1).add(v1.multiply(w2)).add(v1.cross(v2)));
    }

    public Quaternion multiply(double k) {
        return new Quaternion(k*getW(),k*getX(),k*getY(),k*getZ());
    }

    public double normal() {
        return  Math.sqrt(getW()*getW()+getX()*getX()+getY()*getY()+getZ()*getZ());
    }

    public Quaternion normalise() {
        return new Quaternion(this).multiply(1/normal());
    }

    public Quaternion conjugate() {
        return new Quaternion(getW(),-getX(),-getY(),-getZ());
    }

    public Point3D rotate(Point3D point) {
        return (this.multiply(new Quaternion(0,point)).multiply(this.conjugate())).getVector();
    }

    @Deprecated
    public Quaternion power(double power) { /// очень сино подозреваю, что не правильно сделал...
        double phi = Math.acos(getW()/normal());
        Point3D n_ = getVector().normalise().multiply(1/ Math.sin(phi));
        return new Quaternion( Math.cos(power*phi),n_.multiply( Math.sin(power*phi))).multiply( Math.pow(normal(),power));
    }

    @Override
    public String toString() {
        return "Quaternion{" +
                "coordinate=" + coordinate.getA()+" "+coordinate.getB()+" "+coordinate.getC()+" "+coordinate.getD() +
                '}';
    }

}
