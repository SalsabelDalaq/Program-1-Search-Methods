package app;

import java.io.*;
import java.util.*;

public class RouteFinder {
    private static Map<String, List<String>> adjacencyMap = new HashMap<>();
    private static Map<String, double[]> coordinatesMap = new HashMap<>();

    public static void main(String[] args) {
        loadAdjacencies("Adjacencies.txt");
        loadCoordinates("coordinates.csv");

        Scanner scanner = new Scanner(System.in);
        while (true) {
            System.out.println("\nEnter start city: ");
            String start = scanner.nextLine().trim();
            System.out.println("Enter destination city: ");
            String goal = scanner.nextLine().trim();

            if (!adjacencyMap.containsKey(start) || !adjacencyMap.containsKey(goal)) {
                System.out.println("Invalid cities. Please enter valid cities.");
                continue;
            }

            System.out.println("Select search method:");
            System.out.println("1. Brute-Force\n2. BFS\n3. DFS\n4. ID-DFS\n5. Best-First Search\n6. A* Search");
            int choice = scanner.nextInt();
            scanner.nextLine(); // Consume newline

            long startTime = System.nanoTime();
            List<String> path = null;

            switch (choice) {
                case 1 -> path = bruteForce(start, goal);
                case 2 -> path = bfs(start, goal);
                case 3 -> path = dfs(start, goal);
                case 4 -> path = iddfs(start, goal);
                case 5 -> path = bestFirstSearch(start, goal);
                case 6 -> path = aStarSearch(start, goal);
                default -> System.out.println("Invalid choice!");
            }

            long endTime = System.nanoTime();
            if (path != null) {
                System.out.println("Route found: " + path);
                System.out.println("Total distance: " + calculateTotalDistance(path) + " km");
                System.out.println("Execution time: " + (endTime - startTime) / 1e6 + " ms");
            } else {
                System.out.println("No route found.");
            }
        }
    }

    //Load adjacency list from file 
    private static void loadAdjacencies(String filePath) {
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] cities = line.split(" ");
                if (cities.length == 2) {
                    adjacencyMap.putIfAbsent(cities[0], new ArrayList<>());
                    adjacencyMap.putIfAbsent(cities[1], new ArrayList<>());
                    adjacencyMap.get(cities[0]).add(cities[1]);
                    adjacencyMap.get(cities[1]).add(cities[0]);
                }
            }
        } catch (IOException e) {
            System.out.println("Error reading adjacency file: " + e.getMessage());
        }
    }

    // Load city coordinates from file 
    private static void loadCoordinates(String filePath) {
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            br.readLine(); // Skip header
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 3) {
                    String city = parts[0].trim();
                    double lat = Double.parseDouble(parts[1].trim());
                    double lon = Double.parseDouble(parts[2].trim());
                    coordinatesMap.put(city, new double[]{lat, lon});
                }
            }
        } catch (IOException e) {
            System.out.println("Error reading coordinates file: " + e.getMessage());
        }
    }

    // Brute-force search 
    private static List<String> bruteForce(String start, String goal) {
        return bfs(start, goal); // Placeholder: BFS is a common brute-force approach
    }

    //Breadth-First Search (BFS) 
    private static List<String> bfs(String start, String goal) {
        Queue<List<String>> queue = new LinkedList<>();
        Set<String> visited = new HashSet<>();
        queue.add(Collections.singletonList(start));

        while (!queue.isEmpty()) {
            List<String> path = queue.poll();
            String city = path.get(path.size() - 1);
            if (city.equals(goal)) return path;

            if (!visited.contains(city)) {
                visited.add(city);
                for (String neighbor : adjacencyMap.getOrDefault(city, new ArrayList<>())) {
                    List<String> newPath = new ArrayList<>(path);
                    newPath.add(neighbor);
                    queue.add(newPath);
                }
            }
        }
        return null;
    }

    // Depth-First Search (DFS)
    private static List<String> dfs(String start, String goal) {
        Stack<List<String>> stack = new Stack<>();
        Set<String> visited = new HashSet<>();
        stack.push(Collections.singletonList(start));

        while (!stack.isEmpty()) {
            List<String> path = stack.pop();
            String city = path.get(path.size() - 1);
            if (city.equals(goal)) return path;

            if (!visited.contains(city)) {
                visited.add(city);
                for (String neighbor : adjacencyMap.getOrDefault(city, new ArrayList<>())) {
                    List<String> newPath = new ArrayList<>(path);
                    newPath.add(neighbor);
                    stack.push(newPath);
                }
            }
        }
        return null;
    }

    // Iterative Deepening DFS (ID-DFS) 
    private static List<String> iddfs(String start, String goal) {
        int depth = 0;
        while (true) {
            List<String> result = dls(start, goal, depth);
            if (result != null) return result;
            depth++;
        }
    }

    private static List<String> dls(String node, String goal, int depth) {
        if (depth == 0 && node.equals(goal)) return Collections.singletonList(goal);
        if (depth > 0) {
            for (String neighbor : adjacencyMap.getOrDefault(node, new ArrayList<>())) {
                List<String> path = dls(neighbor, goal, depth - 1);
                if (path != null) {
                    List<String> newPath = new ArrayList<>();
                    newPath.add(node);
                    newPath.addAll(path);
                    return newPath;
                }
            }
        }
        return null;
    }

    // Best-First Search (uses heuristic: latitude & longitude) 
    private static List<String> bestFirstSearch(String start, String goal) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(Node::getHeuristic));
        Set<String> visited = new HashSet<>();
        pq.add(new Node(start, null, 0));

        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            String city = currentNode.city;

            if (city.equals(goal)) {
                List<String> path = new ArrayList<>();
                while (currentNode != null) {
                    path.add(currentNode.city);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(path);
                return path;
            }

            if (!visited.contains(city)) {
                visited.add(city);

                for (String neighbor : adjacencyMap.getOrDefault(city, new ArrayList<>())) {
                    if (!visited.contains(neighbor)) {
                        double heuristic = calculateHeuristic(neighbor, goal);
                        pq.add(new Node(neighbor, currentNode, heuristic));
                    }
                }
            }
        }
        return null;
    }

    // A* Search 
    private static List<String> aStarSearch(String start, String goal) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(Node::getTotalCost));
        Set<String> visited = new HashSet<>();
        pq.add(new Node(start, null, 0, calculateHeuristic(start, goal)));

        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            String city = currentNode.city;

            if (city.equals(goal)) {
                List<String> path = new ArrayList<>();
                while (currentNode != null) {
                    path.add(currentNode.city);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(path);
                return path;
            }

            if (!visited.contains(city)) {
                visited.add(city);

                for (String neighbor : adjacencyMap.getOrDefault(city, new ArrayList<>())) {
                    if (!visited.contains(neighbor)) {
                        double g = currentNode.g + haversine(coordinatesMap.get(city)[0], coordinatesMap.get(city)[1], coordinatesMap.get(neighbor)[0], coordinatesMap.get(neighbor)[1]);
                        double h = calculateHeuristic(neighbor, goal);
                        pq.add(new Node(neighbor, currentNode, g, h));
                    }
                }
            }
        }
        return null;
    }

    //Calculate distance using Haversine formula 
    private static double calculateTotalDistance(List<String> path) {
        double distance = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            double[] coord1 = coordinatesMap.get(path.get(i));
            double[] coord2 = coordinatesMap.get(path.get(i + 1));
            if (coord1 != null && coord2 != null) {
                distance += haversine(coord1[0], coord1[1], coord2[0], coord2[1]);
            }
        }
        return distance;
    }

    private static double calculateHeuristic(String city, String goal) {
        double[] coord1 = coordinatesMap.get(city);
        double[] coord2 = coordinatesMap.get(goal);
        if (coord1 != null && coord2 != null) {
            return haversine(coord1[0], coord1[1], coord2[0], coord2[1]);
        }
        return Double.MAX_VALUE;
    }

    private static double haversine(double lat1, double lon1, double lat2, double lon2) {
        final int R = 6371; // Earth radius in km
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c; // Distance in km
    }

    // Node class for Best-First and A* search algorithms 
    private static class Node {
        String city;
        Node parent;
        double g; // Cost from start
        double h; // Heuristic cost to goal

        public Node(String city, Node parent, double h) {
            this(city, parent, 0, h);
        }

        public Node(String city, Node parent, double g, double h) {
            this.city = city;
            this.parent = parent;
            this.g = g;
            this.h = h;
        }

        public double getHeuristic() {
            return h;
        }

        public double getTotalCost() {
            return g + h;
        }
    }
}
