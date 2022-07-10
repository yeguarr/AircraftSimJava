public class Matrix {
    static public class m4x1 {
        private final double a, b, c, d;

        public double getA() {
            return a;
        }

        public double getB() {
            return b;
        }

        public double getC() {
            return c;
        }

        public double getD() {
            return d;
        }

        public m4x1(double a, double b, double c, double d) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
        }
    }
    static public class m4x4 {
        private final double
                aa, ab, ac, ad,
                ba, bb, bc, bd,
                ca, cb, cc, cd,
                da, db, dc, dd;

        public m4x4(double aa, double ab, double ac, double ad, double ba, double bb, double bc, double bd, double ca, double cb, double cc, double cd, double da, double db, double dc, double dd) {
            this.aa = aa;
            this.ab = ab;
            this.ac = ac;
            this.ad = ad;
            this.ba = ba;
            this.bb = bb;
            this.bc = bc;
            this.bd = bd;
            this.ca = ca;
            this.cb = cb;
            this.cc = cc;
            this.cd = cd;
            this.da = da;
            this.db = db;
            this.dc = dc;
            this.dd = dd;
        }

        public m4x4(m4x4 matrix) {
            this.aa = matrix.aa;
            this.ab = matrix.ab;
            this.ac = matrix.ac;
            this.ad = matrix.ad;
            this.ba = matrix.ba;
            this.bb = matrix.bb;
            this.bc = matrix.bc;
            this.bd = matrix.bd;
            this.ca = matrix.ca;
            this.cb = matrix.cb;
            this.cc = matrix.cc;
            this.cd = matrix.cd;
            this.da = matrix.da;
            this.db = matrix.db;
            this.dc = matrix.dc;
            this.dd = matrix.dd;
        }

        public m4x4 multiply(m4x4 second) {
            return new m4x4(
                    this.aa * second.aa + this.ab * second.ba + this.ac * second.ca + this.ad * second.da,
                    this.aa * second.ab + this.ab * second.bb + this.ac * second.cb + this.ad * second.db,
                    this.aa * second.ac + this.ab * second.bc + this.ac * second.cc + this.ad * second.dc,
                    this.aa * second.ad + this.ab * second.bd + this.ac * second.cd + this.ad * second.dd,
                    this.ba * second.aa + this.bb * second.ba + this.bc * second.ca + this.bd * second.da,
                    this.ba * second.ab + this.bb * second.bb + this.bc * second.cb + this.bd * second.db,
                    this.ba * second.ac + this.bb * second.bc + this.bc * second.cc + this.bd * second.dc,
                    this.ba * second.ad + this.bb * second.bd + this.bc * second.cd + this.bd * second.dd,
                    this.ca * second.aa + this.cb * second.ba + this.cc * second.ca + this.cd * second.da,
                    this.ca * second.ab + this.cb * second.bb + this.cc * second.cb + this.cd * second.db,
                    this.ca * second.ac + this.cb * second.bc + this.cc * second.cc + this.cd * second.dc,
                    this.ca * second.ad + this.cb * second.bd + this.cc * second.cd + this.cd * second.dd,
                    this.da * second.aa + this.db * second.ba + this.dc * second.ca + this.dd * second.da,
                    this.da * second.ab + this.db * second.bb + this.dc * second.cb + this.dd * second.db,
                    this.da * second.ac + this.db * second.bc + this.dc * second.cc + this.dd * second.dc,
                    this.da * second.ad + this.db * second.bd + this.dc * second.cd + this.dd * second.dd);
        }

        public m4x1 multiply(m4x1 second) {
            return new m4x1(
                    this.aa * second.getA() + this.ab * second.getB() + this.ac * second.getC() + this.ad * second.getD(),
                    this.ba * second.getA() + this.bb * second.getB() + this.bc * second.getC() + this.bd * second.getD(),
                    this.ca * second.getA() + this.cb * second.getB() + this.cc * second.getC() + this.cd * second.getD(),
                    this.da * second.getA() + this.db * second.getB() + this.dc * second.getC() + this.dd * second.getD());
        }

        public Point3D multiply(Point3D second) {
            return new Point3D(this.multiply(second.getCoordinate()));
        }

        public double[][] getMatrixArray() {
            return new double[][]{
                    {aa, ab, ac, ad},
                    {ba, bb, bc, bd},
                    {ca, cb, cc, cd},
                    {da, db, dc, dd}};
        }

        public m4x4 transpose() {
            return new m4x4(
                    aa,	ba,	ca,	da,
                    ab,	bb,	cb,	db,
                    ac,	bc,	cc,	dc,
                    ad,	bd,	cd,	dd
            );
        }

        public m4x4 inverse() {
            double A2323 = cc * dd - cd * dc;
            double A1323 = cb * dd - cd * db;
            double A1223 = cb * dc - cc * db;
            double A0323 = ca * dd - cd * da;
            double A0223 = ca * dc - cc * da;
            double A0123 = ca * db - cb * da;
            double A2313 = bc * dd - bd * dc;
            double A1313 = bb * dd - bd * db;
            double A1213 = bb * dc - bc * db;
            double A2312 = bc * cd - bd * cc;
            double A1312 = bb * cd - bd * cb;
            double A1212 = bb * cc - bc * cb;
            double A0313 = ba * dd - bd * da;
            double A0213 = ba * dc - bc * da;
            double A0312 = ba * cd - bd * ca;
            double A0212 = ba * cc - bc * ca;
            double A0113 = ba * db - bb * da;
            double A0112 = ba * cb - bb * ca;

            double B1 = bb * A2323 - bc * A1323 + bd * A1223;
            double B2 = ba * A2323 - bc * A0323 + bd * A0223;
            double B3 = ba * A1323 - bb * A0323 + bd * A0123;
            double B4 = ba * A1223 - bb * A0223 + bc * A0123;

            double det = aa * B1 - ab * B2 + ac * B3 - ad * B4;

            return new m4x4(
                       B1 / det,
                     - (ab * A2323 - ac * A1323 + ad * A1223) / det,
                       (ab * A2313 - ac * A1313 + ad * A1213) / det,
                     - (ab * A2312 - ac * A1312 + ad * A1212) / det,
                     -B2 / det,
                       (aa * A2323 - ac * A0323 + ad * A0223) / det,
                     - (aa * A2313 - ac * A0313 + ad * A0213) / det,
                       (aa * A2312 - ac * A0312 + ad * A0212) / det,
                       B3 / det,
                     - (aa * A1323 - ab * A0323 + ad * A0123) / det,
                       (aa * A1313 - ab * A0313 + ad * A0113) / det,
                     - (aa * A1312 - ab * A0312 + ad * A0112) / det,
                     -B4 / det,
                       (aa * A1223 - ab * A0223 + ac * A0123) / det,
                     - (aa * A1213 - ab * A0213 + ac * A0113) / det,
                       (aa * A1212 - ab * A0212 + ac * A0112) / det
                    );
        }
    }
}
