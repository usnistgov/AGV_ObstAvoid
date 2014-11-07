/*
 This is public domain software, however it is preferred
 that the following disclaimers be attached.

 Software Copywrite/Warranty Disclaimer

 This software was developed at the National Institute of Standards and
 Technology by employees of the Federal Government in the course of their
 official duties. Pursuant to title 17 Section 105 of the United States
 Code this software is not subject to copyright protection and is in the
 public domain. This software is experimental. NIST assumes no responsibility
 whatsoever for its use by other parties, and makes no guarantees, expressed
 or implied, about its quality, reliability, or any other characteristic.

 We would appreciate acknowledgement if the software is used. This software can
 be redistributed and/or modified freely provided that any derivative works
 bear some notice that they are derived from it, and any modified
 versions bear some notice that they have been modified.

 */
package splinetest;

import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.lang.reflect.Method;

/**
 * This class implements the Shape interface such that an arbitrary shape may be
 * associated with an arbitrary AffineTransform.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class ShapeWTransform implements Shape {

    public ShapeWTransform(Shape _shape, AffineTransform _affineTransform) {
        this.shape = _shape;
        this.affineTransform = _affineTransform;
    }
    private final Shape shape;
    private final AffineTransform affineTransform;

    /**
     * @return the shape
     */
    public Shape getShape() {
        return shape;
    }

    /**
     * @return the affineTransform
     */
    public AffineTransform getAffineTransform() {
        return affineTransform;
    }

    @Override
    public String toString() {
        return this.getClass().getName() + "[shape=" + shape + ",at=" + affineTransform + "]";
    }

    public static ShapeWTransform valueOf(String s) {
        s = s.trim();
        int shape_index = s.indexOf("[shape=");
        if (shape_index < 0) {
            return null;
        }
        int at_index = s.indexOf(",at=", shape_index + 7);
        if (at_index < at_index) {
            return null;
        }
        int right_sq_brace_index = s.indexOf(']', at_index + 3);
        if (right_sq_brace_index < at_index + 3) {
            return null;
        }
        String shapeString = s.substring(shape_index + 7, at_index).trim();
        String shapeClssname = shapeString;
        int c_index = shapeClssname.indexOf(',');
        if (c_index > 0) {
            shapeClssname = shapeClssname.substring(0, c_index);
        }
        c_index = shapeClssname.indexOf('[');
        if (c_index > 0) {
            shapeClssname = shapeClssname.substring(0, c_index).trim();
        }
        c_index = shapeClssname.indexOf('{');
        if (c_index > 0) {
            shapeClssname = shapeClssname.substring(0, c_index).trim();
        }
        Shape shape = null;
        AffineTransform at = null;
        try {
            Class clss = SplineTestJFrame.class.getClassLoader().loadClass(shapeClssname);
            Method value_of = clss.getMethod("valueOf", String.class);
            if (null != value_of) {
                shape = (Shape) value_of.invoke(null, shapeString);

            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new ShapeWTransform(shape, at);
    }

    @Override
    public Rectangle getBounds() {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.getBounds();
        }
        Rectangle r = shape.getBounds();
        double src[] = {r.x, r.y, r.x + r.width, r.y + r.height};
        double dst[] = new double[4];
        this.affineTransform.transform(src, 0, dst, 0, 2);
        return new Rectangle(
                Math.min((int) dst[0], (int) dst[2]),
                Math.min((int) dst[1], (int) dst[3]),
                Math.abs((int) (dst[2] - dst[0])),
                Math.abs((int) (dst[3] - dst[1])));
    }

    @Override
    public Rectangle2D getBounds2D() {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.getBounds2D();
        }
        Rectangle2D r = shape.getBounds2D();
        double src[] = {r.getX(), r.getY(), r.getX() + r.getWidth(), r.getY() + r.getHeight()};
        double dst[] = new double[4];
        this.affineTransform.transform(src, 0, dst, 0, 2);
        return new Rectangle2Dd(
                Math.min(dst[0], dst[2]),
                Math.min(dst[1], dst[3]),
                Math.abs((dst[2] - dst[0])),
                Math.abs((dst[3] - dst[1])));
    }

    @Override
    public boolean contains(double x, double y) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.contains(x, y);
        }
        double src[] = {x, y};
        double dst[] = new double[2];
        try {
            this.affineTransform.inverseTransform(src, 0, dst, 0, 1);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible");
        }
        return shape.contains(dst[0], dst[1]);
    }

    @Override
    public boolean contains(Point2D p) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.contains(p);
        }
        Point2D inverse_transformed_p = new Point2D.Double();
        try {
            this.affineTransform.inverseTransform(p, inverse_transformed_p);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible.");
        }
        return shape.contains(inverse_transformed_p);
    }

    @Override
    public boolean intersects(double x, double y, double w, double h) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.intersects(x, y, w, h);
        }
        double src[] = {x, y, x + w, y + h};
        double dst[] = new double[4];
        try {
            this.affineTransform.inverseTransform(src, 0, dst, 0, 2);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible");
        }
        return shape.intersects(
                Math.min(dst[0], dst[2]),
                Math.min(dst[1], dst[3]),
                Math.abs(dst[0] - dst[2]),
                Math.abs(dst[3] - dst[1]));
    }

    @Override
    public boolean intersects(Rectangle2D r) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.intersects(r);
        }
        double src[] = {r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()};
        double dst[] = new double[4];
        try {
            this.affineTransform.inverseTransform(src, 0, dst, 0, 2);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible");
        }
        return shape.intersects(
                Math.min(dst[0], dst[2]),
                Math.min(dst[1], dst[3]),
                Math.abs(dst[0] - dst[2]),
                Math.abs(dst[3] - dst[1]));
    }

    @Override
    public boolean contains(double x, double y, double w, double h) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.contains(x, y, w, h);
        }
        double src[] = {x, y, x + w, y + h};
        double dst[] = new double[4];
        try {
            this.affineTransform.inverseTransform(src, 0, dst, 0, 2);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible");
        }
        return shape.contains(
                Math.min(dst[0], dst[2]),
                Math.min(dst[1], dst[3]),
                Math.abs(dst[0] - dst[2]),
                Math.abs(dst[3] - dst[1]));
    }

    @Override
    public boolean contains(Rectangle2D r) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.intersects(r);
        }
        double src[] = {r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()};
        double dst[] = new double[4];
        try {
            this.affineTransform.inverseTransform(src, 0, dst, 0, 2);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalStateException("affineTransform may not be invertible");
        }
        return shape.intersects(
                Math.min(dst[0], dst[2]),
                Math.min(dst[1], dst[3]),
                Math.abs(dst[0] - dst[2]),
                Math.abs(dst[3] - dst[1]));
    }

    @Override
    public PathIterator getPathIterator(AffineTransform _at) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.getPathIterator(_at);
        }
        if (null == _at) {
            return shape.getPathIterator(this.affineTransform);
        }
        AffineTransform fullTransform = new AffineTransform();
        fullTransform.concatenate(this.affineTransform);
        fullTransform.concatenate(_at);
        return shape.getPathIterator(_at);
    }

    @Override
    public PathIterator getPathIterator(AffineTransform _at, double _flatness) {
        if (null == shape) {
            throw new IllegalStateException("shape == null");
        }
        if (null == this.affineTransform) {
            return shape.getPathIterator(_at,_flatness);
        }
        if (null == _at) {
            return shape.getPathIterator(this.affineTransform,_flatness);
        }
        AffineTransform fullTransform = new AffineTransform();
        fullTransform.concatenate(this.affineTransform);
        fullTransform.concatenate(_at);
        return shape.getPathIterator(_at,_flatness);
    }
}
