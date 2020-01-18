import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.io.FileReader;
import java.io.BufferedReader;

public class Dijkstra {

	private Map<String, Vertex> vertexNames;

	public Dijkstra() 
	{
		vertexNames = new HashMap<String, Vertex>();
	}

	public void addVertex(Vertex v) {
		if (vertexNames.containsKey(v.name))
			throw new IllegalArgumentException("Cannot create new vertex (city) with existing name.");
		vertexNames.put(v.name, v);
	}

	public Collection<Vertex> getVertices() {
		return vertexNames.values();
	}

	public Vertex getVertex(String name) {
		return vertexNames.get(name);
	}

	public void addEdge(String nameU, String nameV, Double cost) {
		if (!vertexNames.containsKey(nameU))
			throw new IllegalArgumentException(nameU + " does not exist. Cannot create edge.");
		if (!vertexNames.containsKey(nameV))
			throw new IllegalArgumentException(nameV + " does not exist. Cannot create edge.");
		Vertex sourceVertex = vertexNames.get(nameU);
		Vertex targetVertex = vertexNames.get(nameV);
		Edge newEdge = new Edge(sourceVertex, targetVertex, cost);
		sourceVertex.addEdge(newEdge);
	}

	public void addUndirectedEdge(String nameU, String nameV, double cost) { //undirected means 2 directed
		addEdge(nameU, nameV, cost);
		addEdge(nameV, nameU, cost);
	}

	public double computeEuclideanDistance(double ux, double uy, double vx, double vy) {

		//just distance formula between 2 points 
		double distance = Math.sqrt( Math.pow((vx-ux), 2) + Math.pow((vy-uy), 2));

		return distance; 
	}

	public void computeAllEuclideanDistances() 
	{
	
		for(Map.Entry<String, Vertex> entry: vertexNames.entrySet())
		{

			List<Edge> edgeList = entry.getValue().adjacentEdges;

			//iterate through adjacent cities
			for(int i = 0; i < edgeList.size(); i++)
			{
				Edge e = edgeList.get(i);
				e.distance = computeEuclideanDistance(e.source.x, e.source.y, e.target.x, e.target.y);
			}
	

		} 
	}


	public void doDijkstra(String s) 
	{
		
		for(Vertex v: vertexNames.values())
		{
			v.distance = Double.POSITIVE_INFINITY;
		}
		List<Vertex> visited = new LinkedList<Vertex>();

		List<Vertex> pending = new LinkedList<Vertex>();


		Vertex start = vertexNames.get(s);
		pending.add(start);
		start.distance = 0;
		while(!pending.isEmpty() && visited.size() <vertexNames.size()) 
			Vertex min = findMin(pending);

			visited.add(min);

			for(Edge e : min.adjacentEdges)
			{
				Vertex target = e.target;

				if(target.distance == Double.POSITIVE_INFINITY)
				{
					target.distance = min.distance + e.distance;

					target.prev = e.source;
				}
				
				else if(min.distance + e.distance < target.distance) 											
				{
					target.distance = min.distance + e.distance;

					target.prev = min;
				}

				if(!visited.contains(target) && !pending.contains(target))
				{
					pending.add(target);
				}
			}
		}

		System.out.println();
		
		System.out.println("Distances of all the cities from " + start + " are: ");
		for(Vertex v: visited)
		{
			System.out.println(v + " distance is: " + v.distance);
		}
	}

	public Vertex findMin(List<Vertex> l)
	{
		double min = l.get(0).distance;
		int minIndex = 0;
		for(int i = 1; i < l.size(); i++)
		{
			if(l.get(i).distance < min)
			{
				min = l.get(i).distance;
				minIndex = i;	
			}		
		}
		Vertex v = l.remove(minIndex);
		
		return v;

	}
	
	public List<Edge> getDijkstraPath(String s, String t) {
		doDijkstra(s);

		List<Edge> path = new LinkedList<Edge>();


		Vertex firstVertex = vertexNames.get(s);

		Vertex endVertex = vertexNames.get(t);


		while(endVertex != firstVertex) 
		{	
		
			Vertex previous = endVertex.prev;
			
			if(previous == null)
			{
				List<Edge> emptyPath = new LinkedList<Edge>();
				return emptyPath;
			}
			for(Edge e: previous.adjacentEdges)
			{
				if(e.target == endVertex )
				{
					path.add(0, e);
				}
			}

			endVertex = previous;
		}

		return path;
	}

	public void printAdjacencyList() 
	{
		for (String u : vertexNames.keySet()) {
			StringBuilder sb = new StringBuilder();
			// name of city
			sb.append(u); 
			sb.append(" -> [ ");
			for (Edge e : vertexNames.get(u).adjacentEdges) {
				sb.append(e.target.name);
				sb.append("(");
				sb.append(e.distance);
				sb.append(") ");
			}
			sb.append("]");
			System.out.println(sb.toString());
		}
	}

	public static void main(String[] argv) throws IOException {
		String vertexFile = "cityxy.txt"; 
		String edgeFile = "citypairs.txt";

		Dijkstra dijkstra = new Dijkstra();
		String line;

		// Read in the vertices

		BufferedReader vertexFileBr = new BufferedReader(new FileReader(vertexFile));
		while ((line = vertexFileBr.readLine()) != null) {
			String[] parts = line.split(",");
			if (parts.length != 3) {
				vertexFileBr.close();
				throw new IOException("Invalid line in vertex file " + line);
			}
			String cityname = parts[0];
			int x = Integer.valueOf(parts[1]);
			int y = Integer.valueOf(parts[2]);
			Vertex vertex = new Vertex(cityname, x, y);
			dijkstra.addVertex(vertex);
		}
		vertexFileBr.close();

		BufferedReader edgeFileBr = new BufferedReader(new FileReader(edgeFile));
		while ((line = edgeFileBr.readLine()) != null) {
			String[] parts = line.split(",");
			if (parts.length != 3) {
				edgeFileBr.close();
				throw new IOException("Invalid line in edge file " + line);
			}
			dijkstra.addUndirectedEdge(parts[0], parts[1], Double.parseDouble(parts[2]));
		}
		edgeFileBr.close();

		dijkstra.computeAllEuclideanDistances();
		dijkstra.printAdjacencyList();

		String startCity = "SanFrancisco";
		String endCity = "Boston";

		List<Edge> path = dijkstra.getDijkstraPath(startCity, endCity);
		System.out.println();
		
		System.out.println(startCity + " to " + endCity + " path distances: ");
		double total = 0;
		for(Edge e: path)
		{
			total = total + e.distance;
			System.out.println(e + " distance: " +e.distance);
		}
		
		System.out.println();
		
		System.out.println("TOTAL DISTANCE: " + total);
		System.out.println();
		
		System.out.print("Shortest path between "+startCity+" and "+endCity+": ");
		System.out.println(path);
	}
}
