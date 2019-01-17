/*
* this class represents the list of neighbours for a node.
* This captures other properties like road name, road type and distance.
* */

package roadgraph;

import geography.GeographicPoint;

public class MapEdge {


    //Edges lat and longitude info.
    private GeographicPoint location;
    private String roadName;
    private String roadType;
    private Double length;

    //getter method fo location.
    public GeographicPoint getLocation() {
        return location;
    }

    public MapEdge(GeographicPoint location, String roadName, String roadType,Double length) {
        this.location = location;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public String toString()
    {
        String str = "";
        str+="Lat: " + location.getX()+ ", Lon: " + location.getY()+"\n";

        return str;
    }




}
