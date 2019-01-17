/*
* This class represents the a node in graph.
* This class captures  a Node and its neighours information using MapEdge.
*
* */

package roadgraph;

import geography.GeographicPoint;
import java.util.List;

public class MapNode implements Comparable<MapNode>{

    // Represents the location of the Node with Lat and long.
    private GeographicPoint location;
    //Represents all the neighbour edges with its other properties like road name, length and type of raod.
    private List<MapEdge> edges;

    public Double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    private Double distance;




    //getter method for location
    public GeographicPoint getLocation() {
        return location;
    }

    //setter method for list of neighbhours
    public List<MapEdge> getEdges() {
        return edges;
    }


    public MapNode(GeographicPoint location, List<MapEdge> edges) {
        this.location = location;
        this.edges = edges;
    }



    public String toString()
    {
        String str = "";
        str+="Lat: " + location.getX()+ ", Lon: " + location.getY()+" Dist:"+getDistance()+"\n";

        return str;
    }


    @Override
    public int compareTo(MapNode o) {
        return Double.compare(this.getDistance(),o.getDistance());
    }
}
