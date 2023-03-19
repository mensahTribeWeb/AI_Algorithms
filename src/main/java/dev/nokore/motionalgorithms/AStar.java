package dev.nokore.motionalgorithms;

import java.util.*;

/**
 *This implementation creates a 5x5 grid graph with nodes that have x and y coordinates.
 * The A* algorithm is run on this graph to find the shortest path from the top-left node
 * to the bottom-right node. The heuristic method calculates the Manhattan distance between
 * two nodes, which is used to estimate the distance between a node and the end node.
 * The algorithm uses a priority queue to keep track of the nodes to be explored
 *
 * */

public class AStar {

    // Calculate heuristic value for given node
    public static int heuristic(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    public static void main(String[] args) {
        // Create a graph
        Node[][] graph = new Node[5][5];
        for (int i = 0; i < graph.length; i++) {
            for (int j = 0; j < graph[0].length; j++) {
                graph[i][j] = new Node(i, j);
            }
        }

        // Add neighbors for each node
        for (int i = 0; i < graph.length; i++) {
            for (int j = 0; j < graph[0].length; j++) {
                if (i > 0) {
                    graph[i][j].neighbors.add(graph[i-1][j]);
                }
                if (i < graph.length - 1) {
                    graph[i][j].neighbors.add(graph[i+1][j]);
                }
                if (j > 0) {
                    graph[i][j].neighbors.add(graph[i][j-1]);
                }
                if (j < graph[0].length - 1) {
                    graph[i][j].neighbors.add(graph[i][j+1]);
                }
            }
        }

        // Run A* algorithm
        Node start = graph[0][0];
        Node end = graph[4][4];
        PriorityQueue<Node> openList = new PriorityQueue<Node>((a,b) -> (a.f - b.f));
        HashSet<Node> closedList = new HashSet<Node>();
        openList.add(start);
        while (!openList.isEmpty()) {
            Node current = openList.poll();
            if (current == end) {
                System.out.println("Path found!");
                break;
            }
            closedList.add(current);
            for (Node neighbor : current.neighbors) {
                if (closedList.contains(neighbor)) {
                    continue;
                }
                int tentativeG = current.g + heuristic(current, neighbor);
                if (!openList.contains(neighbor) || tentativeG < neighbor.g) {
                    neighbor.parent = current;
                    neighbor.g = tentativeG;
                    neighbor.f = neighbor.g + heuristic(neighbor, end);
                    if (!openList.contains(neighbor)) {
                        openList.add(neighbor);
                    }
                }
            }
        }

        // Print path from start to end
        Node current = end;
        while (current != null) {
            System.out.println(current);
            current = current.parent;
        }
    }
}

class Node {
    int x;
    int y;
    List<Node> neighbors;
    Node parent;
    int g;
    int f;

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.neighbors = new ArrayList<Node>();
        this.parent = null;
        this.g = 0;
        this.f = 0;
    }

    public String toString() {
        return "(" + this.x + ", " + this.y + ")";
    }
}
