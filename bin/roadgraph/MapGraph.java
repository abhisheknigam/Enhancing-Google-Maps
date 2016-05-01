/**
 *
 */
package roadgraph;

import java.io.PrintWriter;
import java.util.Calendar;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;


import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections of multiple roads.
 * Edges are the roads.
 *
 */
public class MapGraph {

	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;


	/** Create a new empty MapGraph
	 *
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	// For us in DEBUGGING.  Print the Nodes in the graph
	public void printNodes()
	{
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : pointNodeMap.keySet())
		{
			MapNode n = pointNodeMap.get(pt);
			System.out.println(n);
		}
	}

	// For us in DEBUGGING.  Print the Edges in the graph
	public void printEdges()
	{
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges)
		{
			System.out.println(e);
		}

	}

	/** Add a node corresponding to an intersection
	 *
	 * @param latitude The latitude of the location
	 * @param longitude The longitude of the location
	 * */
	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 *
	 * @param location  The location of the intersection
	 */
	public void addVertex(GeographicPoint location)
	{
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
		}
		else {
			System.out.println("Warning: Node at location " + location +
					" already exists in the graph.");
		}

	}

	/** Add an edge representing a segment of a road.
	 * Precondition: The corresponding Nodes must have already been
	 *     added to the graph.
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 */
	public void addEdge(double lat1, double lon1,
						double lat2, double lon2, String roadName, String roadType)
	{
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType) {

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType, double length) {
		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point)
	{
		return pointNodeMap.containsKey(point);
	}



	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName,
			String roadType,  double length)
	{
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}


	/** Returns the nodes in terms of their geographic locations */
	public Collection<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using Breadth First Search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
									Consumer<GeographicPoint> nodeSearched)
	{
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;
	}

	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */

	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;
		for(MapNode mapNode : parentMap.keySet()){
			if(mapNode.equals(current)){
				System.out.println("Actual Distnace" + mapNode.getActualDistance());
			}
		}
		
		if (parentMap.get(current) == null) {
		      return null;
		    }
		    path.add(current.getLocation());
		    while (parentMap.get(current) != null) {
		    current = parentMap.get(current);
		    path.add(current.getLocation());
		    }
		    // Put it into the correct order
		    Collections.reverse(path);
		    
		    return path;
	}

	private String getCurrentHour(){
		Calendar date = Calendar.getInstance();
		String split[] = date.getTime().toString().split(" ");
		return split[3].split(":")[0];
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
        return dijkstraDifferentApproach(start, goal, temp);
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
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		Comparator<MapNode> nodeCompare = new Comparator<MapNode>(){
			@Override
			public int compare(MapNode o1, MapNode o2) {
				if (o1.getDistance() < o2.getDistance()) {
					return -1;
				}
		        if (o1.getDistance() > o2.getDistance()) return 1;
		        return 0;
			}
		};
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new PriorityQueue<MapNode>(nodeCompare);
		HashSet<MapNode> visited = new HashSet<MapNode>();
		startNode.setDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			count ++;
			visited.add(next);
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					if( neighbor.getDistance() > next.getDistance() + getDistanceBetweenNodes(next,neighbor)){
						neighbor.setDistance(next.getDistance() + getDistanceBetweenNodes(next,neighbor));
						parentMap.put(neighbor, next);
						Set<MapNode> set = new HashSet<MapNode>(toExplore);
						if(set.add(neighbor))
							toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);
		System.out.println(count);
		return path;
	}

	public List<GeographicPoint> dijkstraDifferentApproach(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}


		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		List<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		startNode.setDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		while (!toExplore.isEmpty()) {
			next =  getMinimum(toExplore);
			toExplore.remove(next);
			count++;
			visited.add(next);
			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode))
				break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					if (neighbor.getDistance() > next.getDistance() + getDistanceBetweenNodes(next, neighbor)) {
						neighbor.setDistance(next.getDistance() + getDistanceBetweenNodes(next, neighbor));
						parentMap.put(neighbor, next);
						Set<MapNode> set = new HashSet<MapNode>(toExplore);
						if (set.add(neighbor))
							toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}

		// Reconstruct the parent path
		List<GeographicPoint> path = reconstructPath(parentMap, startNode, endNode);
		System.out.println(count);
		return path;
	}
	
	private double getDistanceBetweenNodes(MapNode next, MapNode neighbor) {
		for(MapEdge edge:edges){
			if(edge.getStartNode().equals(next) && edge.getEndNode().equals(neighbor)){
				return edge.getLength();
			}
		}
		return Integer.MAX_VALUE;
	}
	
	private MapEdge getEdge(MapNode next, MapNode neighbor) {
		for(MapEdge edge:edges){
			if(edge.getStartNode().equals(next) && edge.getEndNode().equals(neighbor)){
				return edge;
			}
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
        return aStarSearchDifferentApproach(start, goal, temp);
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
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		Comparator<MapNode> nodeCompare = new Comparator<MapNode>(){
			@Override
			public int compare(MapNode o1, MapNode o2) {
				if (o1.getDistance() < o2.getDistance()) {
					return -1;
				}
		        if (o1.getDistance() > o2.getDistance()) return 1;
		        return 0;
			}
		};
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new PriorityQueue<MapNode>(nodeCompare);
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		startNode.setActualDistance(0);
		startNode.setDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		while (!toExplore.isEmpty()) {
			next = toExplore.poll();
			count++;
			visited.add(next);
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					//getAdditionalLagInDistance()
					if( neighbor.getDistance() + neighbor.getLocation().distance(goal) > next.getDistance() + getDistanceBetweenNodes(next,neighbor) + next.getLocation().distance(goal)){
						neighbor.setDistance(next.getDistance() + getDistanceBetweenNodes(next,neighbor) + neighbor.getLocation().distance(goal));
						neighbor.setActualDistance(next.getDistance() + getDistanceBetweenNodes(next,neighbor));
						parentMap.put(neighbor, next);
						Set<MapNode> set = new HashSet<MapNode>(toExplore);
						if(set.add(neighbor))
							toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);
		System.out.println(count);
		return path;
	}
	
	// The new Astar approach takes into consideration the type of road and the current time to predict the best possible path.
	public List<GeographicPoint> aStarSearchDifferentApproach(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		List<MapNode> toExplore = new LinkedList<MapNode>();

		HashSet<MapNode> visited = new HashSet<MapNode>();
		startNode.setActualDistance(0);
		startNode.setDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		int count = 0;
		while (!toExplore.isEmpty()) {
			//next = toExplore.poll();
			next =  getMinimum(toExplore);
			toExplore.remove(next);
			count++;
			visited.add(next);
			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode))
				break;
			Set<MapNode> neighbors = getNeighbors(next);
			
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					 Double lagDistance = getAdditionalLagInDistance(next, neighbor);
					if (neighbor.getDistance() + neighbor.getLocation().distance(goal) + lagDistance > next.getDistance() + getDistanceBetweenNodes(next, neighbor) + next.getLocation().distance(goal)) {
						neighbor.setDistance(next.getDistance() + getDistanceBetweenNodes(next, neighbor)
								+ neighbor.getLocation().distance(goal) + lagDistance);
						neighbor.setActualDistance(next.getDistance() + getDistanceBetweenNodes(next, neighbor));
						parentMap.put(neighbor, next);
						Set<MapNode> set = new HashSet<MapNode>(toExplore);
						if (set.add(neighbor))
							toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}

		// Reconstruct the parent path
		List<GeographicPoint> path = reconstructPath(parentMap, startNode, endNode);
		System.out.println(count);
		return path;
	}

	// calculates the adjustment in distance based on different parameters 
	private Double getAdditionalLagInDistance(MapNode next, MapNode neighbor) {
		MapEdge currentEdge = null;	
		Double lagDistance = (double) 0;
		for(MapEdge edge:edges){
				if(edge.getStartNode().equals(next) && edge.getEndNode().equals(neighbor)){
					currentEdge = edge;
				}
			}
		lagDistance += getLagThroughRoadType(currentEdge);
		lagDistance += getLagThroughRoadTypeAndTimeOfDay(currentEdge);
		return lagDistance;
	}
	
	// Takes into consideration the type of road to make appropriate prediction.
	private Double getLagThroughRoadType(MapEdge currentEdge){
		Double lagDistance = (double) 0;
		if(currentEdge != null){
			String edgeType = currentEdge.getRoadType();
			Double edgeLength = currentEdge.getLength();
			if(MapEdge.PRIMARY.equalsIgnoreCase(edgeType)){
				lagDistance = -(.25) * edgeLength;
			}else if(MapEdge.SECONDARY.equalsIgnoreCase(edgeType)){
				lagDistance = -(.20) * edgeLength;
			}else if(MapEdge.TERTIARY.equalsIgnoreCase(edgeType)){
				lagDistance = -(.15) * edgeLength;
			}else if(MapEdge.RESIDENTIAL.equalsIgnoreCase(edgeType)){
				lagDistance = -(.10) * edgeLength;
			}
		}
		return lagDistance;
	}
	
	//takes into consideration the time of day to make appropriate consideration
	private Double getLagThroughRoadTypeAndTimeOfDay(MapEdge currentEdge){
		Double lagDistance = (double) 0;
		
		if(currentEdge != null){
			String edgeType = currentEdge.getRoadType();
			Double edgeLength = currentEdge.getLength();
			if(MapEdge.PRIMARY.equalsIgnoreCase(edgeType) && isOfficeHour()){
				lagDistance = -(.25) * edgeLength;
			}else if(MapEdge.SECONDARY.equalsIgnoreCase(edgeType) && isOfficeHour()){
				lagDistance = -(.20) * edgeLength;
			}else if(MapEdge.TERTIARY.equalsIgnoreCase(edgeType) && isOfficeHour()){
				lagDistance = -(.15) * edgeLength;
			}else if(MapEdge.RESIDENTIAL.equalsIgnoreCase(edgeType) && isOfficeHour()){
				lagDistance = -(.10) * edgeLength;
			}
		}
		return lagDistance;
	}

	//Checks for office hours(Rush Hours) and returns boolean true/false
	private boolean isOfficeHour() {
		String hour = getCurrentHour();
		int parsedHour = Integer.parseInt(hour);
		if( (parsedHour >= 8 && parsedHour <= 10) && (parsedHour >= 18 && parsedHour <= 23)){
			return true;
		}
		return false;
	}

	private MapNode getMinimum(List<MapNode> toExplore) {
		Double minimumDistance = (double) Integer.MAX_VALUE;
		MapNode minNode = null;
		for(MapNode mapNode : toExplore){
			if(mapNode.getDistance() < minimumDistance){
				minimumDistance = mapNode.getDistance();
				minNode = mapNode;
			}
		}
		return minNode;
	}

	// main method for testing
	public static void main(String[] args)
	{
		/*  Basic testing
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		*/ 

		// more advanced testing
		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");
//
//		System.out.println("Num nodes: " + theMap.getNumVertices());
//		System.out.println("Num edges: " + theMap.getNumEdges());
//		
//		List<GeographicPoint> route = theMap.aStarSearch(new GeographicPoint(1.0,1.0), 
//												 new GeographicPoint(8.0,-1.0));
//		
		
		
			
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		//List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		//System.out.println(route);
		System.out.println(route2);
				
	}

}

