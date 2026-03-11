# DVRPTW Formulation Used by this Repository

This project implements an Ant Colony System (ACS) algorithm to solve the Dynamic Vehicle Routing Problem with Time Windows (DVRPTW). The formulation and assumptions in the code are summarized below.

- Problem type: DVRPTW (Dynamic Vehicle Routing Problem with Time Windows)
- Underlying static problem: Vehicle Routing Problem with Time Windows (VRPTW) with a single depot and homogeneous vehicles.

Key model elements
- Nodes: A depot (index 0) plus n customer requests. Each request has:
  - id, coordinates (x,y), demand (capacity requirement), ready/earliest time (startWindow), due/latest time (endWindow), service time, available time.
  - availableTime = 0 for apriori (known) requests; >0 for dynamic requests that become known during simulation.
- Vehicles: Homogeneous fleet with a fixed capacity. The maximum fleet size (number of vehicles allowed) is read from the instance.
- Distances: Euclidean / ATT / GEO / ceil (from TSPLIB styles). Distances are computed into a full distance matrix. Distances may be scaled to simulate a fixed-length working day.

Objectives and feasibility
- Objective: Minimize total traveled distance (sum of tour lengths). The algorithm focuses on producing feasible tours respecting time windows and capacity; the main reported metrics are number of vehicles used and total travel distance.
- Time windows: Each customer i must be serviced between its startWindow and endWindow; service consumes serviceTime at arrival. Waiting is implicitly allowed (service starts at max(arrival, startWindow)).
- Capacity constraints: The sum of demands on a route must not exceed vehicle capacity.

Dynamic features (DVRPTW-specific)
- Request availability: Some requests are dynamic and have availableTime > 0; they are not known to the solver until availableTime. The simulation uses a discrete time-slicing of a working day; when a new slice starts, requests whose availableTime <= current simulated time become available.
- Committed nodes: Portions of the best-so-far solution that represent already-served or irrevocably-fixed visit decisions are marked as "committed" and cannot be changed when new nodes arrive. The simulation periodically checks and commits nodes based on service begin times and the current time slice.
- Insertions: When new nodes become available, the algorithm stops the ACS worker thread, marks committed nodes (if any), inserts newly available nodes into the current best solution using insertion heuristics (and nearest-neighbor growth for extra routes), updates tour lengths, then restarts the ACS.

Algorithm
- Main metaheuristic: Ant Colony System (ACS) adapted for VRPTW and dynamic arrivals. The implementation follows classical ACS components: pheromone matrix, nearest-neighbour candidate lists, ants that construct solutions, pheromone updates, and optional local search routines.
- Initialization: The code builds initial feasible tours using nearest-neighbour and insertion heuristics over the apriori known nodes; dynamic nodes are sorted by available time.
- Simulation: A working day (configurable) is simulated by wall-clock time scaled into time windows; the controller monitors elapsed time, reveals dynamic requests, commits nodes, and runs insertion to maintain feasibility as the problem evolves.

Input format
- Plain text instances in `input/` folder. Files are parsed by `DataReader`. Each request line contains fields (8 integers): id, x, y, demand, startWindow, endWindow, serviceTime, availableTime. The depot is included as the first request.

Assumptions & limits
- The algorithm uses a discrete number of time slices and scales instance time windows to a workingDay length; this is used to simulate real-time dynamic arrivals.
- The solver focuses on feasibility and minimizing distance; it does not explicitly model stochastic travel times or multiple depots.
- The current Controller hard-codes the default instance name and dynamic level; you can substitute files or change parameters to run different instances.

References and provenance
- The VRPTW/ACS core is based on ACOTSP code and classical ACS implementations (see header comments in `VRPTW.java`), and this repository extends those ideas to DVRPTW by handling availableTime / dynamic insertions and committed nodes.

Files of interest
- `src/aco/Controller.java` — main simulation loop, time-slicing, commits, insertion on arrival.
- `src/aco/DataReader.java` — input parser and how availableTime is interpreted.
- `src/aco/Request.java` — request attributes (startWindow, endWindow, serviceTime, availableTime).
- `src/aco/VRPTW_ACS.java`, `src/aco/Ants.java`, `src/aco/InsertionHeuristic.java` — ACS algorithm, pheromones, and insertion/local search routines.

If you want this saved in the repo, I can commit DVRPTW_FORMULATION.md for you (or modify its contents). Also I can generate a short CLI note explaining how to run a single-trial simulation on a chosen instance.
