//package dev.nokore.motionalgorithms;
//
//import java.awt.*;
//import java.awt.geom.Point2D;
//import java.util.*;
//import java.util.List;
//
//public class ProbabilisticRoadmap {
//    // Define the size of the workspace
//    private static final int X_MIN = 0;
//    private static final int X_MAX = 100;
//    private static final int Y_MIN = 0;
//    private static final int Y_MAX = 100;
//
//    // Define the number of nodes to generate
//    private static final int NUM_NODES = 1000;
//
//    // Define the robot's radius
//    private static final int ROBOT_RADIUS = 5;
//
//    // Define the minimum distance between nodes
//    private static final int MIN_DISTANCE = 2 * ROBOT_RADIUS;
//
//    // Define the start and goal points
//    private static final Point START = new Point(10, 10);
//    private static final Point GOAL = new Point(90, 90);
//
//    public static void main(String[] args) {
//        // Step 1: Generate the roadmap
//        List<Point> nodes = generateNodes();
//        Map<Point, List<Point>> roadmap = generateRoadmap(nodes);
//
//        // Step 2: Find the shortest path using Dijkstra's algorithm
//        List<Point> shortestPath = dijkstra(roadmap, START, GOAL);
//
//        // Print the shortest path
//        System.out.println("Shortest path:");
//        for (Point point : shortestPath) {
//            System.out.println(point);
//        }
//    }
//
//    // Generates a list of random nodes in the workspace
//    private static List<Point> generateNodes() {
//        List<Point> nodes = new ArrayList<>();
//        Random rand = new Random();
//
//        while (nodes.size() < NUM_NODES) {
//            int x = rand.nextInt(X_MAX - X_MIN) + X_MIN;
//            int y = rand.nextInt(Y_MAX - Y_MIN) + Y_MIN;
//            Point point = new Point(x, y);
//
//            boolean valid = true;
//            for (Point other : nodes) {
//                if (point.distance(other) < MIN_DISTANCE) {
//                    valid = false;
//                    break;
//                }
//            }
//
//            if (valid) {
//                nodes.add(point);
//            }
//        }
//
//        return nodes;
//    }
//
//
//
//    // Generates a roadmap connecting the nodes with valid edges
//    private static <LineSegment> Map<Point, List<Point>> generateRoadmap(List<Point> nodes) {
//        Map<Point, List<Point>> roadmap = new HashMap<>();
//
//        for (Point node : nodes) {
//            List<Point> neighbors = new ArrayList<>();
//
//            for (Point other : nodes) {
//                if (!node.equals(other) && node.distance(other) >= ROBOT_RADIUS) {
//                    boolean validEdge = true;
//
//                    // Check for obstacles between the nodes
//                    LineSegment edge = new LineSegment(node, other);
//                    for (Point obstacle : getObstacles()) {
//                        if (edge.intersectsCircle(obstacle, ROBOT_RADIUS)) {
//                            validEdge = false;
//                            break;
//                        }
//                    }
//
//                    if (validEdge) {
//                        neighbors.add(other);
//                    }
//                }
//            }
//
//            roadmap.put(node, neighbors);
//        }
//
//        return roadmap;
//    }
//
//    private static Point[] getObstacles() {
//        Point[] obstacles = {
//                new Point(30, 30),
//                new Point(40, 40),
//                new Point(60, 60),
//                new Point(70, 70),
//        };
//        return obstacles;
//    }
//
//    // Finds the shortest path between the start and goal using Dijkstra's algorithm
//    private static List<Point> dijkstra(Map<Point, List<Point>> roadmap, Point start, Point goal) {
//        // Initialize the distances and visited nodes
//        Map<Point, Double> distances = new HashMap<>();
//        Map<Point, Point> previousNodes = new HashMap<>();
//        for (Point node : roadmap.keySet()) {
//            distances.put(node, Double.MAX_VALUE);
//            previousNodes.put(node, null);
//        }
//
//        // Set the distance to the start node to 0
//        distances.put(start, 0.0);
//
//        // Create a priority queue to store the nodes to be visited
//        PriorityQueue<Point> queue = new PriorityQueue<>(Comparator.comparing(distances::get));
//        queue.add(start);
//        while (!queue.isEmpty()) {
//            Point current = queue.poll();
//
//            // If the current node is the goal, we're done
//            if (current.equals(goal)) {
//                break;
//            }
//
//            // Iterate over the neighbors of the current node
//            for (Point neighbor : roadmap.get(current)) {
//                Object point2 = null;
//                Point2D point1 = null;
//                double distance = point1.distance((Point2D) point2);
//
//                double newDistance = distances.get(current) + distance;
//
//                // If we've found a shorter path to the neighbor, update the distance and previous node
//                if (newDistance < distances.get(neighbor)) {
//                    distances.put(neighbor, newDistance);
//                    previousNodes.put(neighbor, current);
//                    queue.remove(neighbor);
//                    queue.add(neighbor);
//                }
//            }
//        }
//
//        // Reconstruct the shortest path
//        List<Point> path = new ArrayList<>();
//        Point current = goal;
//        while (current != null) {
//            path.add(current);
//            current = previousNodes.get(current);
//        }
//
//        Collections.reverse(path);
//        return path;
//    }}