/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	/*
	* variables to track and return the number of vertices, number of edges
	* Map to represent the graph using adjacency list
	* */
	private int numVertices;
	private int numEdges;
	private Map<GeographicPoint,MapNode> MapGraphAdjList;
	private static final double ZERODIST = 0;


	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		//Initialize the variables
		numVertices = 0;
		numEdges = 0;
		MapGraphAdjList = new HashMap<GeographicPoint,MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//returning a new set, so the original set cannot be changed.
		return new HashSet<GeographicPoint>(MapGraphAdjList.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	/*
	*  Adding vertices to the graph based on adjacency list representation.
	* */
	public boolean addVertex(GeographicPoint location)
	{
		if(location!=null){
			if(!MapGraphAdjList.containsKey(location)){
				MapGraphAdjList.put(location,null);
				numVertices++;
				return true;
			}
		}

		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */

	/*
	* Adds edges/neighbhours to vertices based on graph data using adjacency list representation
	* */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if(from==null || to==null || roadName==null || roadType==null || length==0){
			throw new IllegalArgumentException();
		}

		MapEdge neighbour = new MapEdge(to,roadName,roadType,length);
		List<MapEdge> edges;

		/*
		* If the adjancency list contains edges, then we add the new edge found in graph data to the existing list.
		* If not we create a new list with new edge and associate with the appropriate vertex.
		* */

		if(MapGraphAdjList.containsKey(from)){
			MapNode mapNode = MapGraphAdjList.get(from);
			if(mapNode!=null){
				edges=mapNode.getEdges();
				edges.add(neighbour);
			}
			else{
                 edges = new ArrayList<MapEdge>();
                 edges.add(neighbour);
				 mapNode = new MapNode(from,edges);

			}
			numEdges++;
			MapGraphAdjList.put(from,mapNode);
		}
		else{
			throw new IllegalArgumentException();
		}

		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		System.out.println("Finding the route using bfs");
		// variable to return the path after finding the goal
		List<GeographicPoint> pathList = new ArrayList<GeographicPoint>();
		//Queue
		Queue<MapNode> mapGraphQueue = new LinkedList<MapNode>();
		//Parent map to store parents to trace back the path.
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		// set of visited nodes while exploring the path.
		Set<MapNode> visitedSet = new HashSet<MapNode>();

		//Cur variable to store the node being explored currently.
		// Cur is of type MapNode which is a data structure used explore the graph represented with adjacency list matrix.
		MapNode cur = MapGraphAdjList.get(start);
		mapGraphQueue.add(cur);
		visitedSet.add(cur);

		// setting to start nodes parent to null,
		// so we can use this condition while tracing the path back from parent map.
		parentMap.put(cur,null);

		/*
		* Start exploring the queue by deque-ing one element at a time
		 * and add the deque-ed elements neighbours back to queue to explore sequentially.
		* */
		while(!mapGraphQueue.isEmpty()){
			cur =mapGraphQueue.remove();
			if(cur==MapGraphAdjList.get(goal)){
				pathList.add(cur.getLocation());
				while(cur!=null){
					cur = parentMap.get(cur);
					if(cur!=null)
						pathList.add(cur.getLocation());
				}
				//Show the path from start to goal.
				Collections.reverse(pathList);
				System.out.println("pathlist.."+pathList);

				return  pathList;
			}

			// exploring the neighbours of vertices adding them to visitedset, parentmap and queue.
			if(cur!=null && cur.getEdges()!=null){
				for(MapEdge neighbour:cur.getEdges() ){
					GeographicPoint location = neighbour.getLocation();
					MapNode vertex = MapGraphAdjList.get(location);
					if(vertex!=null && !visitedSet.contains(vertex)){
						visitedSet.add(vertex);
						parentMap.put(vertex,cur);
						mapGraphQueue.add(vertex);
						nodeSearched.accept(neighbour.getLocation());

					}

				}

			}
		}
		// Hook for visualization.  See writeup.
		System.out.println("pathList:"+parentMap);
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}

	private void printQueue(PriorityQueue<MapNode> mapGraphPriorityQueue) {
		System.out.println("=>"+mapGraphPriorityQueue.toString());

		for(Integer i=0;i<= mapGraphPriorityQueue.size();i++){
		}
	}

	private void printWeightedQueue(PriorityQueue<WeightedMapNode> mapGraphPriorityQueue) {
		System.out.println("=>"+mapGraphPriorityQueue.toString());

		for(Integer i=0;i<= mapGraphPriorityQueue.size();i++){
		}
	}

	private List<GeographicPoint> findPath(Map<MapNode,MapNode> parentMap, MapNode cur,GeographicPoint start) {

		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(cur.getLocation());
		while( !cur.getLocation().equals(start)){
			cur = parentMap.get(cur);
			   path.add(cur.getLocation());
			//System.out.println("Cur:"+cur);

		}
		Collections.reverse(path);
		//System.out.println("Path:"+path);

		return path;


	}

	private void initiateDistancesToEdgesFromStart(Map<GeographicPoint,MapNode> mapGraphAdjList, GeographicPoint start) {

		for(GeographicPoint gp: mapGraphAdjList.keySet()){
			MapNode node = mapGraphAdjList.get(gp);
			if(node!=null){
				//System.out.println("==>"+node.getLocation());
				//System.out.println(Double.POSITIVE_INFINITY);
				node.setDistance(Double.POSITIVE_INFINITY);
				if(node.getLocation().equals(start))
					node.setDistance(ZERODIST);
			}

		}
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		initiateDistancesToEdgesFromStart(MapGraphAdjList, start);

		// variable to return the path after finding the goal
		List<GeographicPoint> pathList = new ArrayList<GeographicPoint>();
		//Queue
		PriorityQueue<MapNode> mapGraphPriorityQueue = new PriorityQueue<MapNode>();
		//Parent map to store parents to trace back the path.
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		// set of visited nodes while exploring the path.
		Set<MapNode> visitedSet = new HashSet<MapNode>();

		//Cur variable to store the node being explored currently.
		// Cur is of type MapNode which is a data structure used explore the graph represented with adjacency list matrix.
		MapNode cur = MapGraphAdjList.get(start);
		mapGraphPriorityQueue.add(cur);

		Iterator itr = mapGraphPriorityQueue.iterator();
		//List<MapNode> NodesExplored = new ArrayList<MapNode>();
		List<MapNode> nodesExplored = new ArrayList<MapNode>();
        //System.out.println("Before While:"+mapGraphPriorityQueue.toArray());
		while(itr.hasNext()){
			//printQueue(mapGraphPriorityQueue);
			cur = mapGraphPriorityQueue.poll();
			System.out.println("Node visited at location using dijkstra:"+cur);
			nodesExplored.add(cur);
			visitedSet.add(cur);

			if(cur.getLocation().equals(goal)){
				System.out.println("Node visited at location using dijkstra:"+nodesExplored.size());
				return findPath(parentMap, cur, start);
			}
			for(MapEdge neighbour: cur.getEdges()){
				double distanceToNeighbour = cur.getLocation().distance(neighbour.getLocation());
				double distanceFromParentToStart = cur.getDistance();
				MapNode neighbourNode = MapGraphAdjList.get(neighbour.getLocation());
				// we need to still explore if these geographich points
				if(neighbourNode==null){
					neighbourNode= new MapNode(neighbour.getLocation(),new ArrayList<MapEdge>());
					neighbourNode.setDistance(0);
					if(!visitedSet.contains(neighbourNode)){
						mapGraphPriorityQueue.add(neighbourNode);
						parentMap.put(neighbourNode,cur);

					}
				}
					double Sum_neighbour_ParentToStart = distanceToNeighbour + distanceFromParentToStart;
					if(Sum_neighbour_ParentToStart<neighbourNode.getDistance()){
						neighbourNode.setDistance(Sum_neighbour_ParentToStart);
						if(!visitedSet.contains(neighbourNode)){
							mapGraphPriorityQueue.add(neighbourNode);
							parentMap.put(neighbourNode,cur);
						}
					}




			}
			nodeSearched.accept(cur.getLocation());

		}
		return null;
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		initiateDistancesToEdgesFromStart(MapGraphAdjList, start);

		// variable to return the path after finding the goal
		List<GeographicPoint> pathList = new ArrayList<GeographicPoint>();
		//Queue
		PriorityQueue<WeightedMapNode> mapGraphPriorityQueue = new PriorityQueue<WeightedMapNode>();

		//Parent map to store parents to trace back the path.
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
        // set of visited nodes while exploring the path.
		Set<MapNode> visitedSet = new HashSet<MapNode>();

		//Cur variable to store the node being explored currently.
		// Cur is of type MapNode which is a data structure used explore the graph represented with adjacency list matrix.
		MapNode cur = MapGraphAdjList.get(start);
		WeightedMapNode curwightedNode = new WeightedMapNode(cur);
		mapGraphPriorityQueue.add(curwightedNode);
        List<WeightedMapNode> nodesExplored = new ArrayList<WeightedMapNode>();
		Iterator itr = mapGraphPriorityQueue.iterator();
		while(itr.hasNext()){
			curwightedNode = mapGraphPriorityQueue.poll();
			cur = curwightedNode.getMapNode();
			visitedSet.add(cur);
			nodesExplored.add(curwightedNode);

			if(cur.getLocation().equals(goal)){
				System.out.println("Node visited at location using A*:"+nodesExplored.size());
				// reached destination. return path.
				return findPath(parentMap, cur, start);
			}
			for(MapEdge neighbour: cur.getEdges()){
				double distanceToNeighbour = cur.getLocation().distance(neighbour.getLocation());
				double distanceFromParentToStart = cur.getDistance();
				double predictedDistanceToGoal = neighbour.getLocation().distance(goal);
				MapNode neighbourNode = MapGraphAdjList.get(neighbour.getLocation());

				if(neighbourNode==null){
					neighbourNode= new MapNode(neighbour.getLocation(),new ArrayList<MapEdge>());
					neighbourNode.setDistance(0);
					if(!visitedSet.contains(neighbourNode)){
						mapGraphPriorityQueue.add(new WeightedMapNode(neighbourNode));
						parentMap.put(neighbourNode,cur);

					}
				}
				double Sum_neighbour_ParentToStart = distanceToNeighbour + distanceFromParentToStart;
				double distance = Sum_neighbour_ParentToStart+predictedDistanceToGoal;
				if(Sum_neighbour_ParentToStart<neighbourNode.getDistance()){
					neighbourNode.setDistance(Sum_neighbour_ParentToStart);
					WeightedMapNode neigbhourwightedNode = new WeightedMapNode(neighbourNode);
					neigbhourwightedNode.setPredictedDistanceToGoal(predictedDistanceToGoal);
					if(!visitedSet.contains(neighbourNode)){
						mapGraphPriorityQueue.add(neigbhourwightedNode);
						parentMap.put(neighbourNode,cur);
						}
					}


			}

			// Hook for visualization.  See writeup.
			nodeSearched.accept(cur.getLocation());

		}





		
		return null;
	}

	public void printGraph(){
		for(GeographicPoint gp : MapGraphAdjList.keySet()){
			System.out.print(gp);
			System.out.print("-->");
			System.out.print(MapGraphAdjList.get(gp));
			System.out.println(";");

		}

	}
	
	public static void main(String[] args)
	{
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		firstMap.bfs(testStart,testEnd);
		//firstMap.printGraph();

		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);

		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		

		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/


		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
}
