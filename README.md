# ProjectAStar
# A* Core Engine

A modular implementation of the **A\* shortest-path search algorithm** written in **C++17**.

This project is developed as part of academic work at the **Ho Chi Minh City University of Technology (HCMUT), Vietnam National University Ho Chi Minh City**, with an emphasis on algorithmic correctness, modular software architecture, and extensibility toward path-planning applications.

The engine separates the generic A\* search procedure from concrete graph representations such as two-dimensional grids. This allows the same search core to operate on different state spaces without coupling the algorithm to any particular application domain.

---

## 1. Overview

A\* is a best-first graph-search algorithm for finding a minimum-cost path between a start node and a goal node.

For each node \(n\), A\* evaluates

\[
f(n) = g(n) + h(n)
\]

where:

- \(g(n)\) is the best known cost from the start node to \(n\),
- \(h(n)\) is a heuristic estimate of the remaining cost from \(n\) to the goal,
- \(f(n)\) is the estimated total cost of a path passing through \(n\).

The algorithm repeatedly selects the node with the lowest \(f\)-value from the frontier and performs edge relaxation until the goal is selected or the search space is exhausted.

This implementation uses **graph-search A\*** and supports reopening previously expanded nodes when a better path is discovered.

---

## 2. Design Goals  

The engine was designed around several principles:

- **Separation of concerns**  
  Search logic, graph structure, grid representation, and heuristic functions are implemented independently.

- **Generic graph search**  
  The A\* core depends only on an abstract graph interface rather than a particular map representation.

- **Correct repeated-state handling**  
  CLOSED nodes may be reopened when a strictly better path is discovered.

- **Deterministic priority ordering**  
  OPEN entries are ordered primarily by \(f\), then by \(h\), and finally by insertion order.

- **Explicit search state**  
  Each search node records its identifier, \(g\)-score, \(h\)-score, and parent.

- **Modern C++17 conventions**  
  The implementation uses RAII, standard-library containers, brace initialization, `std::optional`, `std::function`, and const-correct interfaces.

---

## 3. Project Structure

```text
ProjectAstarCore/
│
├── Core/
│   ├── Node.h
│   ├── Graph.h
│   ├── Astar.h
│   └── Astar.cpp
│
├── Grid/
│   ├── GridNode.h
│   ├── Grid.h
│   └── Grid.cpp
│
├── Heuristics/
│   ├── Heuristics.h
│   └── GridHeuristics.h
│
└── main.cpp      
