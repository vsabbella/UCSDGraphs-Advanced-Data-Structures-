package roadgraph;

import java.util.Map;

public class WeightedMapNode implements Comparable<WeightedMapNode>{

    private double distanceToStart;
    private double predictedDistanceToGoal;
    private MapNode mapNode;


    public double getDistanceToStart() {
        return distanceToStart;
    }

    public void setDistance(double distanceToStart) {
        this.distanceToStart = distanceToStart;
    }


    public WeightedMapNode( MapNode mapNode) {
        this.setDistance(mapNode.getDistance());
        this.mapNode = mapNode;
    }


    public MapNode getMapNode() {
        return mapNode;
    }

    public double getPredictedDistanceToGoal() {
        return predictedDistanceToGoal;
    }

    public void setPredictedDistanceToGoal(double predictedDistanceToGoal) {
        this.predictedDistanceToGoal = predictedDistanceToGoal;
    }

    private double getDistance(){
     return this.getDistanceToStart()+this.getPredictedDistanceToGoal();
    }

    @Override
    public int compareTo(WeightedMapNode o) {
        return Double.compare(this.getDistance(),o.getDistance());
    }

    public String toString()
    {
        String str = "";
        str+=getMapNode().getLocation()+"Dist: "+getDistance()+"\n";

        return str;
    }

}
