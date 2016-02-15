/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
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
	
	Map<GeographicPoint, List<RoadInformation>> graph;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		graph  = new HashMap<GeographicPoint, List<RoadInformation>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return graph.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return graph.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int numOfEdges = 0;
		for(GeographicPoint geographicPoint : graph.keySet()){
			if(graph.get(geographicPoint) != null)
				numOfEdges += graph.get(geographicPoint).size();
		};
		return numOfEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(location != null && !graph.containsKey(location)){
			graph.put(location, null);
			return true;
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		RoadInformation newRoad = new RoadInformation(from, to, roadName, roadType, length);
		List<RoadInformation> roadInfos = graph.get(from);
		if(roadInfos != null){
			roadInfos.add(newRoad);
			graph.put(from, roadInfos);
		}else{
			roadInfos = new ArrayList<RoadInformation>();
			roadInfos.add(newRoad);
			graph.put(from, roadInfos);
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
		List<GeographicPoint> visitedNodes = new LinkedList<>();
		List<GeographicPoint> shortestPath = new LinkedList<>();
		Map<GeographicPoint, GeographicPoint> prev = new HashMap<>();
		
		
		bfs(start, goal, visitedNodes, nodeSearched, prev);
		
		for(GeographicPoint node = goal; node != null; node = prev.get(node)) {
			shortestPath.add(node);
	    }
		Collections.reverse(shortestPath);
		return shortestPath;
	}
	

	private void bfs(GeographicPoint start, GeographicPoint goal, List<GeographicPoint> visitedNodes, Consumer<GeographicPoint> nodeSearched, Map<GeographicPoint, GeographicPoint> prev) {
		List<GeographicPoint> bfsQueue = new LinkedList<>();
		GeographicPoint current = start;
		bfsQueue.add(current);
		visitedNodes.add(current);
		while(!bfsQueue.isEmpty()){
			current = bfsQueue.remove(0);
			
			if(current.equals(goal)){
				visitedNodes.add(current);
				nodeSearched.accept(start);
				break;
			}else if(graph.get(current) != null){
				List<RoadInformation> roads = new ArrayList<RoadInformation>(graph.get(current));
				
				for(RoadInformation road : roads){
					if(!visitedNodes.contains(road.getOtherPoint())){
						visitedNodes.add(road.getOtherPoint());
						bfsQueue.add(road.getOtherPoint());
						prev.put(road.getOtherPoint(), current);
					}
				}
			}
		}
		if (!current.equals(goal)){
	        System.out.println("Destination is not reachable");
	    }
	}
	
	private List<GeographicPoint> findShortestPath(List<GeographicPoint> visitedNodes) {
		List<GeographicPoint> shortestPath = new LinkedList<>();
		int size = visitedNodes.size();
		for(int j = 0 ; j< size ; j++){
			if(visitedNodes.size() > 0){
				GeographicPoint goal = visitedNodes.get(0);
				spanVisitedNodes(visitedNodes, goal, shortestPath);
				shortestPath.add(visitedNodes.remove(0));
			}else{
				break;
			}
		}
		return shortestPath;
	}

	private void spanVisitedNodes(List<GeographicPoint> visitedNodes, GeographicPoint goal, List<GeographicPoint> shortestPath) {
		List<RoadInformation> modRoads = graph.get(goal);
		if(modRoads != null){
			List<RoadInformation> roads = new ArrayList<RoadInformation>(modRoads);
			for(int i = 0 ; i < modRoads.size() ; i++){
				if(shortestPath.contains(modRoads.get(i).getOtherPoint())){
					roads.remove(modRoads.get(i));
				}
			}
			
			for(int i=0; i< roads.size() -1;){
				if(visitedNodes.contains(roads.get(i).getOtherPoint()) && visitedNodes.contains(roads.get(i+1).getOtherPoint()) && visitedNodes.indexOf(roads.get(i).getOtherPoint()) > visitedNodes.indexOf(roads.get(i+1).getOtherPoint())){
					visitedNodes.remove(i);
				}else if(visitedNodes.contains(roads.get(i).getOtherPoint()) && visitedNodes.contains(roads.get(i+1).getOtherPoint()) && visitedNodes.indexOf(roads.get(i).getOtherPoint()) < visitedNodes.indexOf(roads.get(i+1).getOtherPoint())){
					visitedNodes.remove(i+1);
				}else{
					i++;
				}
			}
		
			if(roads.size() == 1){
				int idx = visitedNodes.indexOf(roads.get(0).getOtherPoint());
				for(int k = 1 ; k < idx ; ){
					visitedNodes.remove(k);
					idx --;
				}
			}
		}
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/hollywood_large.map", theMap);
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		//theMap.bfs(new GeographicPoint(34.1080683,-118.3078343), new GeographicPoint(34.1022745,-118.3092607));
		//theMap.bfs(new GeographicPoint(1,1), new GeographicPoint(8,-1));
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
