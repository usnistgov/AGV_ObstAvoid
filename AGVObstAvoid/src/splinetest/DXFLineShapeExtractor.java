/** 
 * This file contains both code developed at NIST which is public domain 
 * and code derived from an example for use of the kabeja library possibly
 * subject to the Apache Software License 2.0.
 * 
 * See 
 *  http://kabeja.sourceforge.net/license.html
 *  and 
 *  http://www-i.nist.gov/admin/pba/styleguide/policyissues.htm#SoftwareDisclaimer
 * 
 */
package splinetest;

import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.io.InputStream;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import org.kabeja.dxf.DXFCircle;


import org.kabeja.dxf.DXFDocument;
import org.kabeja.dxf.DXFLayer;
import org.kabeja.dxf.DXFPolyline;
import org.kabeja.dxf.DXFVertex;
import org.kabeja.dxf.DXFConstants;
import org.kabeja.dxf.DXFEllipse;
import org.kabeja.dxf.DXFLWPolyline;
import org.kabeja.dxf.DXFLine;
import org.kabeja.parser.Parser;
import org.kabeja.parser.DXFParser;
import org.kabeja.parser.ParserBuilder;

/**
 * Class providing utilities for extracting lines and other shapes from
 * AutoCAD DXF (Drawing Interchange Format, or Drawing Exchange Format) files.
 * This should be the only class in this project with kabeja or DXF related dependencies.
 * 
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
public class DXFLineShapeExtractor {

    
   DXFLineShapeExtractor(InputStream in) {
       this.Open(in);
   }
   
   private InputStream in; 
   private Parser parser;
   private DXFDocument doc;
   
    private double scale = 0.1;

    /**
     * Get the value of scale
     *
     * @return the value of scale
     */
    public double getScale() {
        return scale;
    }

    /**
     * Set the value of scale
     *
     * @param scale new value of scale
     */
    public void setScale(double scale) {
        this.scale = scale;
    }

   
   public final void Open(InputStream _in) {
       this.close();
       this.in = _in;
       this.parser = ParserBuilder.createDefaultParser();
       try {
           //parse
            parser.parse(in, DXFParser.DEFAULT_ENCODING);

            //get the documnet and the layer
            doc = parser.getDocument();
//            Iterator it = doc.getDXFHeader().getVarialbeIterator();
//            while(it.hasNext()) {
//                DXFVariable v = (DXFVariable) it.next();
//                System.out.println("v = " + v);
//                System.out.println("v.getName() = " + v.getName());
//                Iterator vit = v.getValueKeyIterator();
//                while(vit.hasNext()) {
//                    String k = (String) vit.next();
//                    System.out.println("k = " + k);
//                    System.out.println("v.getValue(k) = " + v.getValue(k));
//                }
//            }
       } catch(Exception e) {
           e.printStackTrace();
       }
   }
   
   public void close() {
       if(null != parser) {
           parser.releaseDXFDocument();
           parser = null;
       }
       doc = null;
       if(null != in) {
           try { in.close(); } catch(Exception e) {}
           in = null;
       }
   }
   
   public List<String> readLayers() {
       LinkedList<String> layers_list = new LinkedList<>();
       Iterator it = doc.getDXFLayerIterator();
       while(it.hasNext()) {
           layers_list.add(((DXFLayer)(it.next())).getName());
       }
//       System.out.println("layers_list = " + layers_list);
       return layers_list;
   }
   
   public List<Line2Dd> readLinesFromAllLayers() {

       List<String> layers_list = readLayers();
        LinkedList<Line2Dd> line_list = new LinkedList<>();
        for(String layer : layers_list) {
            line_list.addAll(this.readLinesFromLayer(layer));
        }
        return line_list;
   }
   
   public List<Line2Dd> readLinesFromLayer(String layerid) {

        LinkedList<Line2Dd> line_list = new LinkedList<>();
        try {

            DXFLayer layer = doc.getDXFLayer(layerid);
//            System.out.println("layer = " + layer);
//            System.out.println("layer.getName() = " + layer.getName());

            Iterator it = layer.getDXFEntityTypeIterator();
//            System.out.println("Types:");
            while(it.hasNext()) {
                System.out.print(it.next()+",");
            }
//            System.out.println("");
            it = layer.getDXFEntityTypeIterator();
            while (it.hasNext()) {
                String type = (String) it.next();
//                System.out.println("type = " + type);
                //get all polylines from the layer
                List entities = layer.getDXFEntities(type);
//                System.out.println("entities = " + entities);
//                System.out.println("entities.size() = " + entities.size());
                if (type.equals(DXFConstants.ENTITY_TYPE_LINE)) {
                    List<DXFLine> lines = (List<DXFLine>) entities;
                    for (DXFLine dxfl : lines) {
                        Line2Dd l2d = new Line2Dd(
                                dxfl.getStartPoint().getX()*scale,
                                dxfl.getStartPoint().getY()*scale,
                                dxfl.getEndPoint().getX()*scale,
                                dxfl.getEndPoint().getY()*scale);
                        line_list.add(l2d);
                        System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_POLYLINE)) {
                    List<DXFPolyline> plines = (List<DXFPolyline>) entities;
                    for (DXFPolyline pline : plines) {
                        for (int i = 0; i < pline.getVertexCount(); i++) {

                            DXFVertex prev_vertex = pline.getVertex((i  + pline.getVertexCount()- 1)%pline.getVertexCount());
                            DXFVertex vertex = pline.getVertex(i);
//                            System.out.println("vertex = " + vertex);
//                            System.out.println("vertex.getX()*scale = " + vertex.getX()*scale);
//                            System.out.println("vertex.getY()*scale = " + vertex.getY()*scale);
                            Line2Dd l2d = new Line2Dd(
                                    prev_vertex.getX()*scale,
                                    prev_vertex.getY()*scale,
                                    vertex.getX()*scale,
                                    vertex.getY()*scale);
//                            System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                            line_list.add(l2d);
                        }
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_LWPOLYLINE)) {
                    List<DXFLWPolyline> plines = (List<DXFLWPolyline>) entities;
                    for (DXFLWPolyline pline : plines) {
                        for (int i = 0; i < pline.getVertexCount(); i++) {

                            DXFVertex prev_vertex = pline.getVertex((i + pline.getVertexCount()  - 1)%pline.getVertexCount());
                            DXFVertex vertex = pline.getVertex(i);
//                            System.out.println("vertex = " + vertex);
//                            System.out.println("vertex.getX()*scale = " + vertex.getX()*scale);
//                            System.out.println("vertex.getY()*scale = " + vertex.getY()*scale);
                            Line2Dd l2d = new Line2Dd(
                                    prev_vertex.getX()*scale,
                                    prev_vertex.getY()*scale,
                                    vertex.getX()*scale,
                                    vertex.getY()*scale);
//                            System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                            line_list.add(l2d);
                        }
                    }
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        return line_list;
    }
   
   public List<ShapeWTransform> readShapeWTransformFromAllLayers() {

       List<String> layers_list = readLayers();
        LinkedList<ShapeWTransform> shapes_list = new LinkedList<>();
        for(String layer : layers_list) {
            shapes_list.addAll(this.readShapeWTransformFromLayer(layer));
        }
        return shapes_list;
   }
   
   public List<ShapeWTransform> readShapeWTransformFromLayer(String layerid) {

        LinkedList<ShapeWTransform> shapes_list = new LinkedList<>();
        try {

            DXFLayer layer = doc.getDXFLayer(layerid);
            System.out.println("layer = " + layer);
            System.out.println("layer.getName() = " + layer.getName());
            Iterator it = layer.getDXFEntityTypeIterator();
            System.out.println("Types:");
            while(it.hasNext()) {
                System.out.print(it.next()+",");
            }
            System.out.println("");
            it = layer.getDXFEntityTypeIterator();
            while (it.hasNext()) {
                String type = (String) it.next();
                System.out.println("type = " + type);
                //get all polylines from the layer
                List entities = layer.getDXFEntities(type);
                System.out.println("entities = " + entities);
                System.out.println("entities.size() = " + entities.size());
                if (type.equals(DXFConstants.ENTITY_TYPE_LINE)) {
                    List<DXFLine> lines = (List<DXFLine>) entities;
                    for (DXFLine dxfl : lines) {
                        Line2Dd l2d = new Line2Dd(
                                dxfl.getStartPoint().getX()*scale,
                                dxfl.getStartPoint().getY()*scale,
                                dxfl.getEndPoint().getX()*scale,
                                dxfl.getEndPoint().getY()*scale);
                        shapes_list.add(new ShapeWTransform(l2d,null));
                        System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_POLYLINE)) {
                    List<DXFPolyline> plines = (List<DXFPolyline>) entities;
                    for (DXFPolyline pline : plines) {
                        for (int i = 0; i < pline.getVertexCount(); i++) {

                            DXFVertex prev_vertex = pline.getVertex((i +pline.getVertexCount()- 1)%pline.getVertexCount());
                            DXFVertex vertex = pline.getVertex(i);
                            System.out.println("vertex = " + vertex);
                            System.out.println("vertex.getX()*scale = " + vertex.getX()*scale);
                            System.out.println("vertex.getY()*scale = " + vertex.getY()*scale);
                            Line2Dd l2d = new Line2Dd(
                                    prev_vertex.getX()*scale,
                                    prev_vertex.getY()*scale,
                                    vertex.getX()*scale,
                                    vertex.getY()*scale);
                            System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                            shapes_list.add(new ShapeWTransform(l2d,null));
                        }
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_LWPOLYLINE)) {
                    List<DXFLWPolyline> plines = (List<DXFLWPolyline>) entities;
                    for (DXFLWPolyline pline : plines) {
                        for (int i = 0; i < pline.getVertexCount(); i++) {

                            DXFVertex prev_vertex = pline.getVertex((i +pline.getVertexCount() - 1)%pline.getVertexCount());
                            DXFVertex vertex = pline.getVertex(i);
                            System.out.println("vertex = " + vertex);
                            System.out.println("vertex.getX()*scale = " + vertex.getX()*scale);
                            System.out.println("vertex.getY()*scale = " + vertex.getY()*scale);
                            Line2Dd l2d = new Line2Dd(
                                    prev_vertex.getX()*scale,
                                    prev_vertex.getY()*scale,
                                    vertex.getX()*scale,
                                    vertex.getY()*scale);
                            System.out.println("l2d = [(" + l2d.x1 + "," + l2d.y1 + "),(" + l2d.x2 + "," + l2d.y2 + ")]");
                            shapes_list.add(new ShapeWTransform(l2d,null));
                        }
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_CIRCLE)) {
                    List<DXFCircle> dfx_circles = (List<DXFCircle>) entities;
                    for (DXFCircle dfx_circle : dfx_circles) {
                            Ellipse2D.Double ellipse2d = 
                                    new Ellipse2D.Double();
                            ellipse2d.x = (dfx_circle.getCenterPoint().getX()- dfx_circle.getRadius())*scale;
                            ellipse2d.y = (dfx_circle.getCenterPoint().getY()- dfx_circle.getRadius())*scale;
                            ellipse2d.width = dfx_circle.getRadius()*scale*2.0;
                            ellipse2d.height = dfx_circle.getRadius()*scale*2.0;
                            System.out.println("l2d = [(" + ellipse2d.x + "," + ellipse2d.y + "," + ellipse2d.width + "," + ellipse2d.height + ")]");
//                            AffineTransform at = new AffineTransform();
//                            at.translate(dfx_circle.getCenterPoint().getX()*scale,dfx_circle.getCenterPoint().getY()*scale);
//                            at.rotate(Math.atan2(dfx_circle.getMajorAxisDirection().getY()*scale,
//                                    dfx_circle.getMajorAxisDirection().getX()*scale));
                            shapes_list.add(new ShapeWTransform(ellipse2d,null));
                    }
                }
                if (type.equals(DXFConstants.ENTITY_TYPE_ELLIPSE)) {
                    List<DXFEllipse> dfx_ellipses = (List<DXFEllipse>) entities;
                    for (DXFEllipse dfx_ellipse : dfx_ellipses) {
                            Ellipse2D.Double ellipse2d = 
                                    new Ellipse2D.Double();
                            ellipse2d.x = - dfx_ellipse.getHalfMajorAxisLength()*scale;
                            ellipse2d.y = - dfx_ellipse.getHalfMajorAxisLength()*scale*dfx_ellipse.getRatio();
                            ellipse2d.width = dfx_ellipse.getHalfMajorAxisLength()*scale*2.0;
                            ellipse2d.height = dfx_ellipse.getHalfMajorAxisLength()*scale*dfx_ellipse.getRatio()*2.0;
                            System.out.println("l2d = [(" + ellipse2d.x + "," + ellipse2d.y + "),(" + ellipse2d.width + "," + ellipse2d.y + ")]");
                            AffineTransform at = new AffineTransform();
                            at.translate(dfx_ellipse.getCenterPoint().getX()*scale,dfx_ellipse.getCenterPoint().getY()*scale);
                            at.rotate(Math.atan2(dfx_ellipse.getMajorAxisDirection().getY()*scale,
                                    dfx_ellipse.getMajorAxisDirection().getX()*scale));
                            shapes_list.add(new ShapeWTransform(ellipse2d,at));
                    }
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        return shapes_list;
    }
}