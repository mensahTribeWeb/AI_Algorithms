package dev.nokore.motionalgorithms;

/**
 * Dijkstra's Algorithm is a special case of A*. This algorithm begins at the starting node and examines all the adjacent edges until it finds the shortest,
 * then moves to that node and repeats until it reaches the target node in the graph.
 *
 *Dijkstra's algorithm is a greedy algorithm used to find the shortest path between
 *  two nodes in a weighted graph. The algorithm works by maintaining a set of
 *  nodes and their tentative distances from the starting node. The algorithm
 *  initially sets the tentative distance of the starting node to zero and the
 *  tentative distances of all other nodes to infinity.
 *  Then, it selects the node with the smallest tentative distance and visits
 *  all of its neighbors. For each neighbor, the algorithm calculates
 *  the tentative distance as the sum of the current node's tentative
 *  distance and the weight of the edge connecting the two nodes.
 *  If the calculated tentative distance is smaller than the neighbor's
 *  current tentative distance, the neighbor's tentative distance is updated.
 *  This process continues until the target node is reached or all reachable nodes
 *  have been visited.
 *
 *  code:
 *  This code creates a DijkstraAlgorithm class that allows you to add edges to a
 *  graph and compute the shortest path between two nodes using Dijkstra's Algorithm.
 *  The shortestPath method takes two arguments, the starting node and the ending
 *  node, and returns a list of integers representing the nodes in the shortest path
 *  from the start to the end. The implementation uses a priority queue to keep track
 *  of the nodes to visit and their distances from the start node
 *
 *
 * The key difference between Dijkstra's algorithm and A* algorithm is the way they
 * the next node to visit. Dijkstra's algorithm selects the node with the smallest
 * distance from the source node, while A* algorithm selects the node that minimizes
 * the sum of the distance from the source node and a heuristic function that
 * estimates the remaining distance to the target node.
 * */

import java.util.*;

public class DijkstraAlgorithm {
    private final Map<Integer, Map<Integer, Integer>> adjacencyList;

    public DijkstraAlgorithm(int numVertices) {
        adjacencyList = new HashMap<>(numVertices);
    }

    public void addEdge(int from, int to, int weight) {
        adjacencyList.computeIfAbsent(from, k -> new HashMap<>()).put(to, weight);
        adjacencyList.computeIfAbsent(to, k -> new HashMap<>()).put(from, weight);
    }

    public List<Integer> shortestPath(int start, int end) {
        final Map<Integer, Integer> distances = new HashMap<>();
        final Map<Integer, Integer> parents = new HashMap<>();
        final PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingInt(distances::get));

        // Initialize distances and add start node to priority queue
        distances.put(start, 0);
        pq.add(start);

        // Iterate until queue is empty or end node is found
        while (!pq.isEmpty()) {
            final int current = pq.poll();

            // If end node is found, break out of loop
            if (current == end) {
                break;
            }

            // Check all neighbors of current node
            for (Map.Entry<Integer, Integer> neighborEntry : adjacencyList.getOrDefault(current, Collections.emptyMap()).entrySet()) {
                final int neighbor = neighborEntry.getKey();
                final int edgeWeight = neighborEntry.getValue();
                final int distanceThroughCurrent = distances.get(current) + edgeWeight;

                // If new distance through current node is shorter than previous distance, update distance and parent
                if (distanceThroughCurrent < distances.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    distances.put(neighbor, distanceThroughCurrent);
                    parents.put(neighbor, current);
                    pq.add(neighbor);
                }
            }
        }

        // If end node was not found, return an empty list
        if (!parents.containsKey(end)) {
            return Collections.emptyList();
        }

        // Build path from start to end node
        final LinkedList<Integer> path = new LinkedList<>();
        int current = end;
        while (current != start) {
            path.addFirst(current);
            current = parents.get(current);
        }
        path.addFirst(start);

        return path;
    }

    public static void main(String[] args) {
        final DijkstraAlgorithm graph = new DijkstraAlgorithm(6);
        graph.addEdge(0, 1, 5);
        graph.addEdge(0, 2, 3);
        graph.addEdge(1, 2, 2);
        graph.addEdge(1, 3, 6);
        graph.addEdge(2, 3, 7);
        graph.addEdge(2, 4, 4);
        graph.addEdge(2, 5, 2);
        graph.addEdge(3, 4, -1);
        graph.addEdge(4, 5, -2);

        final List<Integer> shortestPath = graph.shortestPath(0, 5);
        if (shortestPath.isEmpty()) {
            System.out.println("No path found");
        } else {
            System.out.println("Shortest path: " + shortestPath);
        }
    }
}

