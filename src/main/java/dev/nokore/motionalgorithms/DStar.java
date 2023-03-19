package dev.nokore.motionalgorithms;
/**
 *
 *D* (pronounced "D star") is a pathfinding algorithm that is an extension of the A*
 *  algorithm. The primary difference between D* and A* is that D* is an incremental
 *  search algorithm. Incremental search algorithms work by exploring the graph as
 *  necessary, rather than all at once. In the case of D*, this means that the
 *  algorithm only re-explores parts of the graph that have changed since the last
 *  search.
 *
 * The basic idea of D* is that it maintains a map of the entire graph and uses that
 * to compute the shortest path from the starting node to the goal node. As the
 * search progresses, the algorithm updates this map to reflect any changes that
 * may have occurred in the graph. When the graph changes, D* re-explores only the
 * affected parts of the graph to compute the shortest path.
 *
 * D* works by maintaining two values for each node in the graph: the g-value and
 * the rhs-value. The g-value is the actual cost of getting from the starting node
 * to that node, while the rhs-value is the estimated cost of getting from that node
 * to the goal node. D* uses these values to determine which nodes to explore next.
 *
 * At each step, D* selects the node with the smallest f-value,
 * where f(n) = min(g(n), rhs(n)) + h(n), and expands it.
 * The algorithm then updates the g-value of any nodes affected by the change and
 * recomputes the rhs-value of any nodes that have a higher g-value than the node
 * that was just expanded.
 *
 * D* is particularly useful in situations where the graph changes frequently
 * or where the cost of moving between nodes changes dynamically.
 *
 * code:
 * D* algorithm is a complex algorithm that involves maintaining a priority queue,
 * calculating edge costs and heuristic values, and performing various updates to
 * the graph based on changes in the environment.
 *
 * Here's a sample implementation of D* algorithm in Java for a 2D grid map:
 * */
import java.util.*;

public class DStar {
    private static final double kM = 1.0;

    private final int[][] map;
    private final int[][] cost;
    private final int n;
    private final int m;
    private int startX;
    private int startY;
    private int goalX;
    private int goalY;
    private int[][] rhs;
    private int[][] g;
    private Map<Node, Double> u;
    private PriorityQueue<Node> open;

    public DStar(int[][] map, int startX, int startY, int goalX, int goalY) {
        this.map = map;
        this.n = map.length;
        this.m = map[0].length;
        this.cost = new int[n][m];
        this.startX = startX;
        this.startY = startY;
        this.goalX = goalX;
        this.goalY = goalY;

        // Initialize rhs and g values for all nodes
        this.rhs = new int[n][m];
        this.g = new int[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                rhs[i][j] = Integer.MAX_VALUE;
                g[i][j] = Integer.MAX_VALUE;
            }
        }
        rhs[goalX][goalY] = 0;
        g[goalX][goalY] = 0;

        // Initialize priority queue with goal node
        this.u = new HashMap<>();
        this.open = new PriorityQueue<>((n1, n2) -> {
            double k1 = calculateKey(n1);
            double k2 = calculateKey(n2);
            return Double.compare(k1, k2);
        });
        Node goalNode = new Node(goalX, goalY);
        u.put(goalNode, 0.0);
        open.offer(goalNode);
    }

    public List<Node> findPath() {
        // Calculate path until the start node is reached or the priority queue is empty
        while (!open.isEmpty()) {
            Node curr = open.peek();
            if (curr.equals(new Node(startX, startY))) {
                break;
            }
            open.poll();
            double k_old = calculateKey(curr);
            double k_new = calculateKeyPrime(curr);
            if (k_old < k_new) {
                u.put(curr, k_old);
            } else if (g[curr.x][curr.y] > rhs[curr.x][curr.y]) {
                g[curr.x][curr.y] = rhs[curr.x][curr.y];
                for (Node neighbor : getNeighbors(curr)) {
                    updateVertex(neighbor);
                }
            } else {
                g[curr.x][curr.y] = Integer.MAX_VALUE;
                for (Node neighbor : getNeighbors(curr)) {
                    updateVertex(neighbor);
                }
                updateVertex(curr);
            }
        }

        // Build path by following the minimum cost edges
        List<Node> path = new ArrayList<>();
        Node curr = new Node(startX, startY);
        while (!curr.equals(new Node(goalX, goalY))) {
            path.add(curr);
            curr = getMinimumSuccessor(curr);
        }
        path.add(curr);
        return path;
    }

    private List<Node> getNeighbors(Node curr) {
        List<Node> neighbors = new ArrayList<>();
        int x = curr.x;
        int y = curr.y;
        if (x > 0 && map[x - 1][y] == 0) {
            neighbors.add(new Node(x - 1, y));
        }
        if (y > 0 && map[x][y - 1] == 0) {
            neighbors.add(new Node(x, y - 1));
        }
        if (x < n -

                1 && map[x + 1][y] == 0) {
            neighbors.add(new Node(x + 1, y));
        }
        if (y < m - 1 && map[x][y + 1] == 0) {
            neighbors.add(new Node(x, y + 1));
        }
        return neighbors;
    }

    private double calculateKey(Node node) {
        int x = node.x;
        int y = node.y;
        int minRhs = getMinimumRhs(node);
        return Math.min(g[x][y], minRhs) + heuristic(node, new Node(startX, startY)) + kM;
    }

    private double calculateKeyPrime(Node node) {
        int x = node.x;
        int y = node.y;
        int minRhs = getMinimumRhs(node);
        return Math.min(g[x][y], minRhs) + heuristic(node, new Node(startX, startY));
    }

    private int getMinimumRhs(Node node) {
        int x = node.x;
        int y = node.y;
        int minRhs = Integer.MAX_VALUE;
        for (Node neighbor : getNeighbors(node)) {
            int cost = getCost(node, neighbor);
            minRhs = Math.min(minRhs, g[neighbor.x][neighbor.y] + cost);
        }
        return minRhs;
    }

    private int getCost(Node from, Node to) {
        if (from.x == to.x && from.y == to.y) {
            return 0;
        }
        return cost[to.x][to.y];
    }

    private double heuristic(Node from, Node to) {
        return Math.sqrt(Math.pow(from.x - to.x, 2) + Math.pow(from.y - to.y, 2));
    }

    private void updateVertex(Node node) {
        if (!node.equals(new Node(goalX, goalY))) {
            rhs[node.x][node.y] = getMinimumRhs(node);
        }
        if (open.contains(node)) {
            open.remove(node);
        }
        if (g[node.x][node.y] != rhs[node.x][node.y]) {
            u.put(node, calculateKey(node));
            open.offer(node);
        }
    }

    private Node getMinimumSuccessor(Node node) {
        double minCost = Double.POSITIVE_INFINITY;
        Node minNode = null;
        for (Node neighbor : getNeighbors(node)) {
            double cost = g[neighbor.x][neighbor.y] + getCost(node, neighbor);
            if (cost < minCost) {
                minCost = cost;
                minNode = neighbor;
            }
        }
        return minNode;
    }

}