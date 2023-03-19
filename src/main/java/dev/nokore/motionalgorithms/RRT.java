package dev.nokore.motionalgorithms;

import java.util.*;

public class RRT {

    // Define the dimensions of the environment
    static final int ENV_WIDTH = 100;
    static final int ENV_HEIGHT = 100;

    // Define the maximum distance a new node can be from an existing node
    static final int MAX_NODE_DISTANCE = 10;

    // Define the maximum number of nodes in the tree
    static final int MAX_TREE_SIZE = 10000;

    // Define the starting and goal positions
    static final int START_X = 10;
    static final int START_Y = 10;
    static final int GOAL_X = 90;
    static final int GOAL_Y = 90;

    public static void main(String[] args) {
        // Create the tree with the starting position as the root node
        Node rootNode = new Node(START_X, START_Y, null);
        List<Node> tree = new ArrayList<Node>();
        tree.add(rootNode);

        // Iterate until the goal position is reached or the maximum tree size is exceeded
        while (tree.size() < MAX_TREE_SIZE) {
            // Generate a random configuration in the environment
            int x = (int) (Math.random() * ENV_WIDTH);
            int y = (int) (Math.random() * ENV_HEIGHT);

            // Find the nearest node in the tree to the random configuration
            Node nearestNode = null;
            double nearestDist = Double.POSITIVE_INFINITY;
            for (Node node : tree) {
                double dist = Math.sqrt(Math.pow(x - node.x, 2) + Math.pow(y - node.y, 2));
                if (dist < nearestDist) {
                    nearestNode = node;
                    nearestDist = dist;
                }
            }

            // If the random configuration is within the maximum distance from the nearest node, add it to the tree
            if (nearestDist <= MAX_NODE_DISTANCE) {
                Node newNode = new Node(x, y, nearestNode);
                nearestNode.addChild(newNode);
                tree.add(newNode);

                // If the new node is close to the goal position, exit the loop and return the path
                if (Math.sqrt(Math.pow(x - GOAL_X, 2) + Math.pow(y - GOAL_Y, 2)) <= MAX_NODE_DISTANCE) {
                    List<Node> path = new ArrayList<Node>();
                    Node node = newNode;
                    while (node != null) {
                        path.add(node);
                        node = node.parent;
                    }
                    Collections.reverse(path);
                    System.out.println("Path found:");
                    for (Node n : path) {
                        System.out.println(n);
                    }
                    return;
                }
            }
        }

        System.out.println("Maximum tree size exceeded, no path found.");
    }

    // Define a class to represent a node in the tree
    static class Node {
        int x, y;
        Node parent;
        List<Node> children;

        public Node(int x, int y, Node parent) {
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.children = new ArrayList<Node>();
        }

        public void addChild(Node child) {
            children.add(child);
        }

        public String toString() {
            return "(" + x + ", " + y + ")";
        }
    }

}

