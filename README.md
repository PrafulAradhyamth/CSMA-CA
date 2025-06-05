# CSMA/CA Wireless Network Simulation in MATLAB

This repository contains a MATLAB implementation of a CSMA/CA (Carrier Sense Multiple Access with Collision Avoidance) wireless network simulation. The simulation models how multiple nodes contend for a shared wireless medium to transmit data, incorporating key CSMA/CA features like DIFS, SIFS, backoff, and optional RTS/CTS handshaking.

---

## Description

This simulation is built using a discrete-event simulation approach in MATLAB. It tracks events in a time-ordered queue to simulate the concurrent actions of multiple sending nodes and a central receiving node (router). It allows for the analysis of network performance metrics such as **throughput** and **number of collisions** under different conditions, including varying numbers of nodes and the presence or absence of the RTS/CTS mechanism.

---

## How to Run the Simulation

1.  **Save the `RecievingNodes` Class:**
    MATLAB requires class definitions to be in their own `.m` files. Save the `RecievingNodes` class code (provided in a separate immersive in our conversation) into a file named `RecievingNodes.m`. Make sure this file is in the same directory as the main simulation script.

2.  **Save the Main Simulation Script:**
    Save the main MATLAB code for the CSMA/CA simulation (the `csma_ca_matlab` Canvas) into a file named, for example, `CSMACASimulation.m`.

3.  **Open MATLAB:**
    Launch MATLAB and navigate to the directory where you saved both `RecievingNodes.m` and `CSMACASimulation.m`.

4.  **Run Desired Analysis:**
    The `CSMACASimulation.m` script contains several functions for running simulations and analyses. You can call them directly from the MATLAB Command Window. By default, only `Simple_RUN()` is uncommented at the end of the script.

    * **`Simple_RUN()`**: Executes a single simulation run with default parameters and prints the total packets transferred to the command window.
    * **`Analyze_CTS_RTS()`**: Runs multiple simulations to compare **throughput** when RTS/CTS is enabled versus disabled, across varying numbers of nodes. It then plots the results.
    * **`Analyze_Collisions()`**: Similar to `Analyze_CTS_RTS()`, but focuses on comparing the **number of collisions** with and without RTS/CTS across varying numbers of nodes.
    * **`Analyze_Coverage_Area()`**: Investigates how throughput changes with different **coverage ranges** (simulating varied interference scenarios) with and without RTS/CTS.

    To run a specific analysis, uncomment the corresponding function call at the end of the `CSMACASimulation.m` file and then run the script (e.g., by typing `CSMACASimulation` in the Command Window and pressing Enter, or clicking the "Run" button in the MATLAB editor).

    **Example:** To run the throughput analysis, uncomment `Analyze_CTS_RTS();` and comment out `Simple_RUN();` at the bottom of `CSMACASimulation.m`.

---

## Simulation Parameters

The following global parameters can be adjusted within the `CSMACASimulation.m` file to modify simulation behavior:

* **`PACKETS`**: (Global counter) Total number of successfully transferred packets.
* **`RECEIVING_PROCESSING_TIME`**: Time units for the router to process a frame.
* **`SENDING_NODE_INVOKE_TIME`**: Time at which each sending node is "invoked" to start attempting transmission (defaults to 0 for simultaneous start).
* **`SIM_TIME`**: Total simulation duration in time units.
* **`NUMBER_OF_NODES`**: Initial number of sending nodes in the network (can be varied in analysis functions).
* **`ATTEMPT_LIMIT`**: Maximum number of re-attempts a node will make for a packet before giving up.
* **`DIFS`**: Distributed Interframe Space duration.
* **`CONTENTION_WINDOW_SLOT`**: Duration of a single slot in the contention window for backoff calculation.
* **`SIFS`**: Short Interframe Space duration.
* **`ACK_TIMER`**: Timeout duration for receiving an acknowledgment.
* **`CARIER_SENSING_DURATION`**: Time duration a node senses the carrier before making a decision.
* **`CTS_RTS`**: A boolean flag (`true`/`false`) to enable or disable the Request-To-Send/Clear-To-Send (RTS/CTS) handshake mechanism.
* **`COVERAGE_RANGE`**: Defines how many nodes share the same "local sensing area." A value of `1` means each node has its own distinct sensing area (no hidden terminal issues due to carrier sensing range), while higher values can simulate shared interference domains.
* **`Data_senders`**: (Global counter) Tracks the number of nodes currently attempting to transmit data simultaneously, used for collision detection.
* **`LAST_TIME`**: (Global variable) Records the simulation time of the last successful packet transfer.
* **`COLLISIONS`**: (Global counter) Total number of detected unique collision events.
* **`PREVIOUS_COLLISION_TIME`**: Helps ensure unique collision events are counted.
* **`channel_busy`**: (Global flag) Indicates if the entire channel is currently busy with a transmission.

---

## Simulation Logic

The simulation operates based on a discrete-event approach:

* **Event Queue**: A central queue stores upcoming events, sorted by their time of occurrence.
* **Event Processing**: The simulation progresses by taking the next event from the queue, advancing the simulation time to that event's scheduled time, and executing the logic associated with that event type.
* **Node States**: Each node maintains its state, including its current attempt number (`ATTEMPT`), chosen backoff value (`R`), and flags indicating if it's waiting for CTS or ACK.
* **Carrier Sensing**: Nodes check `getLocalSensingResult` (which depends on `COVERAGE_NODES_IDLE` and `COVERAGE_RANGE`) and `channel_busy` (global channel state) to determine if the medium is clear before transmitting.
* **RTS/CTS Handshake (if enabled)**:
    * Sender sends an RTS frame.
    * Router receives RTS and, if no collision, sends a CTS frame.
    * Sender receives CTS and then sends the Data frame.
    * Router receives Data and sends an ACK frame.
    * Sender receives ACK, indicating success.
* **Collision Handling**: If multiple nodes transmit simultaneously, a collision is detected. Nodes increment their attempt counter and perform an exponential backoff before re-attempting transmission.

---

## Analysis Plots

The `Analyze_CTS_RTS()`, `Analyze_Collisions()`, and `Analyze_Coverage_Area()` functions generate bar plots to visualize the simulation results:

* **Throughput Plots**: Show the average packets per unit time as the number of nodes or coverage area changes.
* **Collision Plots**: Illustrate the average number of collisions as the number of nodes changes.

These plots help in understanding the impact of various CSMA/CA parameters, especially the RTS/CTS mechanism and network density, on network performance.
