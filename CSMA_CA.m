% WIRELESS NETWORKS, 2023
% MEMBERS - SAGAR SUMAN, ARPIT KUMAR, PUNEET

% CSMA/CA SIMULATIONS - MATLAB Translation

% --- Global Simulation Parameters ---
% Declare global variables to be accessible across functions
global PACKETS;
global RECEIVING_PROCESSING_TIME;
global SENDING_NODE_INVOKE_TIME;
global SIM_TIME;
global NUMBER_OF_NODES;
global ATTEMPT_LIMIT;
global DIFS;
global CONTENTION_WINDOW_SLOT;
global SIFS;
global ACK_TIMER;
global CARIER_SENSING_DURATION;
global CTS_RTS;
global COVERAGE_RANGE;
global COVERAGE_NODES_IDLE; % Boolean array indicating local sensing state
global Data_senders; % Counter for active senders for collision detection
global LAST_TIME; % Last successful packet transmission time
global COLLISIONS; % Total collisions count
global PREVIOUS_COLLISION_TIME; % To count unique collision events
global channel_busy; % Global boolean indicating if the channel is currently busy with transmission
global recieving_nodes_instance; % Global instance of the RecievingNodes class
global node_states; % Struct array to store persistent state for each node

% Initialize default values 
PACKETS = 0;
RECEIVING_PROCESSING_TIME = 1;
SENDING_NODE_INVOKE_TIME = 0;
SIM_TIME = 20000; % Total simulation time
NUMBER_OF_NODES = 10; % Default number of nodes
ATTEMPT_LIMIT = 10; % Max re-attempts for a packet
DIFS = 5; % Distributed Interframe Space
CONTENTION_WINDOW_SLOT = 1; % Duration of a contention window slot
SIFS = 2; % Short Interframe Space
ACK_TIMER = 2; % Time for acknowledgement
CARIER_SENSING_DURATION = 2; % How long a node senses the carrier before acting
CTS_RTS = true; % Enable/Disable CTS/RTS mechanism
COVERAGE_RANGE = 1; % Nodes covered by a single sensing area (1 means each node has its own)

% --- Helper Functions for Event Queue Management ---

function events_queue = add_event_to_queue(events_queue, event)
    % Adds a new event to the queue while maintaining sorted order by time.
    if isempty(events_queue)
        events_queue = event;
        return;
    end
    
    % Find insertion point to maintain sorted order
    insert_idx = find([events_queue.time] > event.time, 1, 'first');
    if isempty(insert_idx)
        % Add to end if event is later than all existing events
        events_queue(end+1) = event;
    else
        % Insert at the correct position
        events_queue = [events_queue(1:insert_idx-1), event, events_queue(insert_idx:end)];
    end
end

function events_queue = add_events_to_queue(events_queue, new_events)
    % Adds multiple new events to the queue.
    % Check if new_events is empty or not
    if ~isempty(new_events)
        % Ensure events_queue is initialized as a struct array if it's empty
        if isempty(events_queue) && isstruct(new_events)
            events_queue = new_events(1); % Initialize with the first new event
            if length(new_events) > 1
                events_queue(end+1:end+length(new_events)-1) = new_events(2:end);
            end
        else
            for i = 1:length(new_events)
                events_queue = add_event_to_queue(events_queue, new_events(i));
            end
        end
    end
end

function [event, events_queue] = get_next_event_from_queue(events_queue)
    % Retrieves the earliest event from the queue and removes it.
    event = events_queue(1);
    events_queue(1) = []; % Remove the first event
end

function event_struct = create_event(time, type, node_id, packet_type, current_attempt, R_value)
    % Creates a standard event struct for the simulation.
    % Arguments:
    %   time: The simulation time at which this event is scheduled.
    %   type: A string describing the event type (e.g., 'node_initiate_send', 'difs_complete').
    %   node_id: The ID of the node initiating or affected by this event (0 for router events).
    %   packet_type: 0 for RTS, 1 for Data frame, -1 for other internal signals.
    %   current_attempt: The current re-attempt number for the node (for backoff calculation).
    %   R_value: The random backoff value chosen by the node.
    event_struct = struct(...
        'time', time, ...
        'type', type, ...
        'node_id', node_id, ...
        'packet_type', packet_type, ...
        'current_attempt', current_attempt, ...
        'R_value', R_value);
end


% --- Carrier Sensing Functions ---

function result = getLocalSensingResult(node_id)
    % Returns the carrier sensing result for a specific node's coverage area.
    global COVERAGE_NODES_IDLE;
    global COVERAGE_RANGE;
    % Calculate the index for the coverage area based on node ID and COVERAGE_RANGE
    idx = floor((node_id - 1) / COVERAGE_RANGE) + 1;
    result = COVERAGE_NODES_IDLE(idx);
end

function setLocalSensingResult(node_id, value)
    % Sets the carrier sensing result for a specific node's coverage area.
    global COVERAGE_NODES_IDLE;
    global COVERAGE_RANGE;
    idx = floor((node_id - 1) / COVERAGE_RANGE) + 1;
    COVERAGE_NODES_IDLE(idx) = value;
end


% --- Main Event Processing Logic ---

function [updated_node_states, new_events] = process_event(event, current_sim_time, recieving_nodes_instance, node_states)
    % This function processes a single event from the event queue.
    % It updates global simulation state and schedules new events.
    global PACKETS;
    global DIFS;
    global CONTENTION_WINDOW_SLOT;
    global SIFS;
    global ACK_TIMER;
    global ATTEMPT_LIMIT;
    global RECEIVING_PROCESSING_TIME;
    global CTS_RTS;
    global CARIER_SENSING_DURATION;
    global channel_busy;
    global LAST_TIME;
    
    % Initialize new_events as an empty struct array
    new_events = struct('time', {}, 'type', {}, 'node_id', {}, 'packet_type', {}, 'current_attempt', {}, 'R_value', {}); 
    updated_node_states = node_states; % Create a copy to modify node states

    node_id = event.node_id;
    if node_id > 0 % Only for sender node events, get node-specific state
        node_attempt = updated_node_states(node_id).ATTEMPT;
        node_R = updated_node_states(node_id).R;
        % Flags for tracking sender's waiting state
        is_rts_sent_and_waiting_for_cts = updated_node_states(node_id).is_rts_sent_and_waiting_for_cts;
        is_data_sent_and_waiting_for_ack = updated_node_states(node_id).is_data_sent_and_waiting_for_ack;
    else
        % For router events (node_id = 0), these variables are not relevant
        node_attempt = -1; node_R = -1; % Placeholder
        is_rts_sent_and_waiting_for_cts = false;
        is_data_sent_and_waiting_for_ack = false;
    end

    switch event.type
        case 'node_initiate_send'
            % Event: A node is trying to initiate a send attempt.
            disp(['NODE ', num2str(node_id), ' COMES AT ', num2str(current_sim_time), ': WAITING FOR IDLE']);
            % Check if the channel is idle both locally and globally.
            if ~getLocalSensingResult(node_id) || channel_busy
                % Channel is not idle, schedule a re-check after CARIER_SENSING_DURATION
                next_event_time = current_sim_time + CARIER_SENSING_DURATION;
                new_events(end+1) = create_event(next_event_time, 'node_initiate_send', node_id, -1, node_attempt, node_R);
            else
                % Channel is idle, proceed to wait for DIFS.
                difs_delay = randi([DIFS, 2*DIFS]); % Random DIFS as per original Python logic
                next_event_time = current_sim_time + difs_delay;
                new_events(end+1) = create_event(next_event_time, 'difs_complete', node_id, -1, node_attempt, node_R);
            end

        case 'difs_complete'
            % Event: Node has completed its DIFS wait.
            % Now, choose a random backoff value.
            node_R = randi([0, 2^node_attempt]); % Exponential backoff: 0 to 2^k - 1
            updated_node_states(node_id).R = node_R; % Update node's state

            contention_window_delay = node_R * CONTENTION_WINDOW_SLOT;
            next_event_time = current_sim_time + contention_window_delay;
            new_events(end+1) = create_event(next_event_time, 'backoff_complete', node_id, -1, node_attempt, node_R);

        case 'backoff_complete'
            % Event: Node has completed its backoff.
            % Check channel again before attempting transmission.
            if ~getLocalSensingResult(node_id) || channel_busy
                % Channel became busy during backoff, increment attempt, and re-backoff.
                node_attempt = node_attempt + 1;
                updated_node_states(node_id).ATTEMPT = node_attempt;
                disp(['NODE ', num2str(node_id), ' AT ATTEMPT OF k = ', num2str(node_attempt), ', CHANNEL BUSY AFTER BACKOFF at ', num2str(current_sim_time)]);
                if node_attempt < ATTEMPT_LIMIT
                    % Re-schedule 'node_initiate_send' to restart the process.
                    new_events(end+1) = create_event(current_sim_time, 'node_initiate_send', node_id, -1, node_attempt, node_R);
                else
                    disp(['NODE ', num2str(node_id), ' FAILED TO SEND AFTER ', num2str(ATTEMPT_LIMIT), ' ATTEMPTS at ', num2str(current_sim_time)]);
                    % Node gives up, no more events for this node.
                end
            else
                % Channel is idle, proceed to send RTS or Data frame.
                if CTS_RTS
                    % RTS/CTS enabled: Send RTS first.
                    disp(['RTS SENT BY NODE ', num2str(node_id), ' at ', num2str(current_sim_time)]);
                    setLocalSensingResult(node_id, false); % Node's local sensing area becomes busy
                    channel_busy = true; % Global channel becomes busy with RTS transmission

                    % Schedule RTS arrival at router after transmission time.
                    rts_arrival_time = current_sim_time + RECEIVING_PROCESSING_TIME;
                    new_events(end+1) = create_event(rts_arrival_time, 'rts_arrival_at_router', node_id, 0, node_attempt, node_R);

                    % Node now waits for CTS from router.
                    updated_node_states(node_id).is_rts_sent_and_waiting_for_cts = true;
                    % Schedule a timeout for the sender to check for CTS.
                    % Timeout includes RTS transmission, SIFS, CTS transmission, and an ACK_TIMER margin.
                    cts_timeout_time = current_sim_time + RECEIVING_PROCESSING_TIME + SIFS + RECEIVING_PROCESSING_TIME + ACK_TIMER;
                    new_events(end+1) = create_event(cts_timeout_time, 'cts_timeout_check', node_id, -1, node_attempt, node_R);

                else % CTS/RTS disabled: Send Data Frame directly.
                    disp(['FRAME SENT BY NODE ', num2str(node_id), ' at ', num2str(current_sim_time)]);
                    setLocalSensingResult(node_id, false); % Node's local sensing area becomes busy
                    channel_busy = true; % Global channel becomes busy with Data transmission

                    % Schedule Data frame arrival at router after transmission time.
                    data_arrival_time = current_sim_time + RECEIVING_PROCESSING_TIME;
                    new_events(end+1) = create_event(data_arrival_time, 'data_arrival_at_router', node_id, 1, node_attempt, node_R);

                    % Node now waits for implicit ACK (by observing channel within a timeout).
                    updated_node_states(node_id).is_data_sent_and_waiting_for_ack = true;
                    % Schedule a timeout for the sender to check for implicit ACK.
                    ack_timeout_time = current_sim_time + RECEIVING_PROCESSING_TIME + ACK_TIMER;
                    new_events(end+1) = create_event(ack_timeout_time, 'ack_timeout_check', node_id, -1, node_attempt, node_R);
                end
            end

        case 'rts_arrival_at_router'
            % Event: An RTS frame has arrived at the router.
            [outcome_router, time_taken_router] = recieving_nodes_instance.receive_frame_at_router(event.node_id, event.packet_type, current_sim_time);
            setLocalSensingResult(event.node_id, true); % Router's local area becomes free after receiving RTS

            if strcmp(outcome_router.type, 'collision')
                % Collision detected at router while receiving RTS.
                % Signal collision back to the sender.
                new_events(end+1) = create_event(current_sim_time, 'collision_response', event.node_id, -1, event.current_attempt, event.R_value);
                channel_busy = false; % Channel becomes free after collision
            elseif strcmp(outcome_router.type, 'send_cts')
                disp(['ROUTER SENDING CTS at ', num2str(current_sim_time)]);
                % Router sends CTS. Schedule CTS arrival at sender.
                cts_arrival_time = current_sim_time + outcome_router.delay_for_cts;
                new_events(end+1) = create_event(cts_arrival_time, 'cts_arrival_at_sender', event.node_id, -1, event.current_attempt, event.R_value);
                % Router releases channel after sending CTS.
                new_events(end+1) = create_event(cts_arrival_time, 'router_channel_release', 0, -1, -1, -1); % node_id 0 for router
            end

        case 'data_arrival_at_router'
            % Event: A Data frame has arrived at the router.
            [outcome_router, time_taken_router] = recieving_nodes_instance.receive_frame_at_router(event.node_id, event.packet_type, current_sim_time);
            setLocalSensingResult(event.node_id, true); % Router's local area becomes free after receiving Data

            if strcmp(outcome_router.type, 'collision')
                % Collision detected at router while receiving Data.
                new_events(end+1) = create_event(current_sim_time, 'collision_response', event.node_id, -1, event.current_attempt, event.R_value);
                channel_busy = false; % Channel becomes free after collision
            elseif strcmp(outcome_router.type, 'send_ack')
                disp(['ROUTER SENDING ACK at ', num2str(current_sim_time)]);
                % Router sends ACK. Schedule ACK arrival at sender.
                ack_arrival_time = current_sim_time + outcome_router.delay_for_ack;
                new_events(end+1) = create_event(ack_arrival_time, 'ack_arrival_at_sender', event.node_id, -1, event.current_attempt, event.R_value);
                % Router releases channel after sending ACK.
                new_events(end+1) = create_event(ack_arrival_time, 'router_channel_release', 0, -1, -1, -1);
            elseif strcmp(outcome_router.type, 'data_received_no_ack_sim') && ~CTS_RTS
                % If no RTS/CTS, data was received, and the channel is now free.
                % The sender's timeout will determine if it considers the transmission successful.
                channel_busy = false;
            end

        case 'cts_arrival_at_sender'
            % Event: A CTS frame has arrived at the sender.
            if updated_node_states(node_id).is_rts_sent_and_waiting_for_cts
                % CTS received by sender, proceed to send Data Frame.
                disp(['CTS RECEIVED BY NODE ', num2str(node_id), ' at ', num2str(current_sim_time)]);
                updated_node_states(node_id).is_rts_sent_and_waiting_for_cts = false; % No longer waiting for CTS
                
                % Wait SIFS before sending data.
                next_event_time = current_sim_time + SIFS;
                new_events(end+1) = create_event(next_event_time, 'send_data_frame', node_id, -1, node_attempt, node_R);
            end

        case 'cts_timeout_check'
            % Event: Sender's timeout for CTS has expired.
            if updated_node_states(node_id).is_rts_sent_and_waiting_for_cts
                % CTS not received within timeout, consider it a collision for RTS.
                disp(['NODE ', num2str(node_id), ' CTS TIMEOUT (RTS Collision) at ', num2str(current_sim_time)]);
                updated_node_states(node_id).is_rts_sent_and_waiting_for_cts = false;
                new_events(end+1) = create_event(current_sim_time, 'collision_response', node_id, -1, node_attempt, node_R);
            end

        case 'send_data_frame'
            % Event: Node sends a Data frame (only in CTS/RTS scenario after CTS).
            disp(['FRAME SENT BY NODE ', num2str(node_id), ' at ', num2str(current_sim_time)]);
            setLocalSensingResult(node_id, false); % Node's local sensing area becomes busy
            channel_busy = true; % Global channel becomes busy with Data transmission

            % Schedule Data frame arrival at router.
            data_arrival_time = current_sim_time + RECEIVING_PROCESSING_TIME;
            new_events(end+1) = create_event(data_arrival_time, 'data_arrival_at_router', node_id, 1, node_attempt, node_R);

            % Node now waits for ACK from router.
            updated_node_states(node_id).is_data_sent_and_waiting_for_ack = true;
            % Schedule a timeout for the sender to check for ACK.
            ack_timeout_time = current_sim_time + RECEIVING_PROCESSING_TIME + SIFS + RECEIVING_PROCESSING_TIME + ACK_TIMER;
            new_events(end+1) = create_event(ack_timeout_time, 'ack_timeout_check', node_id, -1, node_attempt, node_R);

        case 'ack_arrival_at_sender'
            % Event: An ACK frame has arrived at the sender.
            if updated_node_states(node_id).is_data_sent_and_waiting_for_ack
                disp(['ACK RECEIVED BY NODE ', num2str(node_id), ' at ', num2str(current_sim_time)]);
                LAST_TIME = current_sim_time; % Record time of successful packet
                PACKETS = PACKETS + 1; % Increment successful packets count
                updated_node_states(node_id).is_data_sent_and_waiting_for_ack = false; % No longer waiting for ACK
                % Packet successfully sent, this node is considered done for this simulation run.
                % No new events are scheduled for this node's current packet.
            end

        case 'ack_timeout_check'
            % Event: Sender's timeout for ACK has expired.
            if updated_node_states(node_id).is_data_sent_and_waiting_for_ack
                % ACK not received within timeout, consider it a collision for Data Frame.
                disp(['NODE ', num2str(node_id), ' ACK TIMEOUT (Data Frame Collision) at ', num2str(current_sim_time)]);
                updated_node_states(node_id).is_data_sent_and_waiting_for_ack = false;
                new_events(end+1) = create_event(current_sim_time, 'collision_response', node_id, -1, node_attempt, node_R);
            end

        case 'collision_response'
            % Event: Signals that a collision was detected or inferred by the sender.
            node_attempt = node_attempt + 1; % Increment re-attempt count
            updated_node_states(node_id).ATTEMPT = node_attempt;

            if node_attempt < ATTEMPT_LIMIT
                % Calculate backoff time and re-attempt.
                Tb = node_R * RECEIVING_PROCESSING_TIME; % Backoff time based on R and processing time
                disp(['NODE ', num2str(node_id), ' AT ATTEMPT OF k = ', num2str(node_attempt), ', BACKOFF TIME = ', num2str(Tb), ' at ', num2str(current_sim_time)]);
                next_event_time = current_sim_time + Tb;
                % Re-schedule 'node_initiate_send' to restart the process after backoff.
                new_events(end+1) = create_event(next_event_time, 'node_initiate_send', node_id, -1, node_attempt, node_R);
            else
                disp(['NODE ', num2str(node_id), ' FAILED TO SEND AFTER ', num2str(ATTEMPT_LIMIT), ' ATTEMPTS at ', num2str(current_sim_time)]);
                % Node gives up, no more events for this node.
            end

        case 'router_channel_release'
            % Event: Router has finished sending CTS/ACK and releases the channel.
            recieving_nodes_instance.router_release_channel();
            % Channel is now free for other transmissions.

        otherwise
            disp(['Unknown event type: ', event.type]);
    end
end


% --- Simulation Setup and Run Functions ---

function run_simulation(current_sim_time, initial_nodes)
    % Initializes and runs the discrete-event simulation.
    % current_sim_time: Initial time for the simulation (usually 0).
    % initial_nodes: The number of nodes to simulate for this run.
    
    global PACKETS;
    global Data_senders;
    global LAST_TIME;
    global COLLISIONS;
    global PREVIOUS_COLLISION_TIME;
    global channel_busy;
    global NUMBER_OF_NODES;
    global COVERAGE_RANGE;
    global COVERAGE_NODES_IDLE;
    global recieving_nodes_instance;
    global node_states;
    global SIM_TIME;
    global RECEIVING_PROCESSING_TIME;
    global SENDING_NODE_INVOKE_TIME;

    % Reset global variables for each simulation run to ensure independence
    PACKETS = 0;
    Data_senders = 0;
    LAST_TIME = 0;
    COLLISIONS = 0;
    PREVIOUS_COLLISION_TIME = -1;
    channel_busy = false; % Channel starts free

    NUMBER_OF_NODES = initial_nodes; % Set the number of nodes for this specific run
    
    % Initialize COVERAGE_NODES_IDLE based on current NUMBER_OF_NODES and COVERAGE_RANGE
    num_coverage_areas = floor(NUMBER_OF_NODES / COVERAGE_RANGE) + 1;
    COVERAGE_NODES_IDLE = true(1, num_coverage_areas); % All coverage areas start idle

    % Initialize the receiving node instance
    recieving_nodes_instance = RecievingNodes(RECEIVING_PROCESSING_TIME);

    % Initialize node_states struct array for each sender node
    node_states = struct();
    for i = 1:NUMBER_OF_NODES
        node_states(i).ATTEMPT = 0;
        node_states(i).R = 0;
        node_states(i).is_rts_sent_and_waiting_for_cts = false;
        node_states(i).is_data_sent_and_waiting_for_ack = false;
    end

    % Initialize the event queue
    events_queue = [];

    % Schedule initial events for each node to start attempting to send a packet.
    % Nodes are "invoked" at SENDING_NODE_INVOKE_TIME.
    for i = 1:NUMBER_OF_NODES
        initial_event = create_event(current_sim_time + SENDING_NODE_INVOKE_TIME, 'node_initiate_send', i, -1, 0, 0);
        events_queue = add_event_to_queue(events_queue, initial_event);
    end

    % Main simulation loop
    while ~isempty(events_queue) && current_sim_time < SIM_TIME
        % Get the next earliest event from the queue
        [current_event, events_queue] = get_next_event_from_queue(events_queue);
        current_sim_time = current_event.time; % Advance simulation time to the event time

        % Check if simulation time limit has been reached
        if current_sim_time >= SIM_TIME
            break;
        end

        % Process the current event
        [node_states, new_events_generated] = process_event(current_event, current_sim_time, recieving_nodes_instance, node_states);
        % Add any new events generated by the process_event function to the queue
        events_queue = add_events_to_queue(events_queue, new_events_generated);
    end
end


function Simple_RUN()
    % Runs a single simulation with default parameters and prints the result.
    global SIM_TIME;
    global PACKETS;
    global NUMBER_OF_NODES;

    disp(' ');
    disp('Starting Simple CSMA/CA Simulation...');
    
    % Run the simulation from time 0, with the default NUMBER_OF_NODES
    run_simulation(0, NUMBER_OF_NODES);

    disp(' ');
    disp(['TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
    disp('Simple Simulation Finished.');
end


% --- Analysis Functions for Plotting ---

function Analyze_CTS_RTS()
    % Analyzes throughput with and without CTS/RTS mechanism across varying number of nodes.
    global PACKETS;
    global NUMBER_OF_NODES;
    global CTS_RTS;
    global LAST_TIME;
    global SIM_TIME;

    disp(' ');
    disp('Starting Throughput Analysis: CTS/RTS vs No CTS/RTS');
    
    node_range = 1:30; % Number of nodes to simulate
    num_iterations = 30; % Number of iterations for averaging each data point

    % --- Without CTS/RTS ---
    disp('WITHOUT CTS/RTS SIMULATION');
    without_cts_rts_throughput = zeros(1, length(node_range));
    for i = 1:length(node_range)
        current_nodes = node_range(i);
        fprintf('\n--- For Number of Nodes = %d ---\n', current_nodes);
        CTS_RTS = false; % Disable CTS/RTS for this block

        iteration_throughputs = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, current_nodes); % Run simulation for current_nodes
            
            % Calculate throughput (packets per unit time)
            if LAST_TIME > 0
                iteration_throughputs(j) = PACKETS / LAST_TIME;
            else
                iteration_throughputs(j) = 0; % No packets transferred
            end
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        % Store average throughput for the current number of nodes
        without_cts_rts_throughput(i) = sum(iteration_throughputs) / num_iterations;
    end

    % --- With CTS/RTS ---
    disp(' ');
    disp('WITH CTS/RTS SIMULATION');
    with_cts_rts_throughput = zeros(1, length(node_range));
    for i = 1:length(node_range)
        current_nodes = node_range(i);
        fprintf('\n--- For Number of Nodes = %d ---\n', current_nodes);
        CTS_RTS = true; % Enable CTS/RTS for this block

        iteration_throughputs = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, current_nodes); % Run simulation for current_nodes
            
            if LAST_TIME > 0
                iteration_throughputs(j) = PACKETS / LAST_TIME;
            else
                iteration_throughputs(j) = 0;
            end
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        with_cts_rts_throughput(i) = sum(iteration_throughputs) / num_iterations;
    end

    disp(' ');
    disp('WITHOUT CTS/RTS Throughput:');
    disp(without_cts_rts_throughput);
    disp('WITH CTS/RTS Throughput:');
    disp(with_cts_rts_throughput);

    % Plotting the results
    figure('Position', [100, 100, 800, 500]); % Set figure size
    bar_width = 0.4; % Width of each bar
    
    % Adjust x-coordinates for side-by-side bars
    x_no_cts = node_range - bar_width/2;
    x_with_cts = node_range + bar_width/2;

    bar(x_no_cts, without_cts_rts_throughput, bar_width, 'FaceColor', [0.8 0 0], 'EdgeColor', 'k', 'DisplayName', 'Without RTS/CTS'); hold on; % Maroon-like
    bar(x_with_cts, with_cts_rts_throughput, bar_width, 'FaceColor', [0 0 0.8], 'EdgeColor', 'k', 'DisplayName', 'With RTS/CTS'); hold off; % Blue-like

    xlabel('Number of Nodes');
    ylabel('Throughput (Packets/Unit Time)');
    title('Throughput vs. Number of Nodes');
    legend('Location', 'best');
    grid on;
    xticks(node_range); % Ensure x-axis ticks align with node numbers
    set(gca, 'FontSize', 10); % Set font size for axes
    
    disp('Throughput Analysis Finished.');
end

function Analyze_Collisions()
    % Analyzes the number of collisions with and without CTS/RTS across varying number of nodes.
    global PACKETS;
    global NUMBER_OF_NODES;
    global CTS_RTS;
    global COLLISIONS;
    global SIM_TIME;

    disp(' ');
    disp('Starting Collision Analysis: CTS/RTS vs No CTS/RTS');
    
    node_range = 1:30; % Number of nodes to simulate
    num_iterations = 30; % Number of iterations for averaging each data point

    % --- Without CTS/RTS ---
    disp('WITHOUT CTS/RTS SIMULATION');
    without_cts_rts_collisions = zeros(1, length(node_range));
    for i = 1:length(node_range)
        current_nodes = node_range(i);
        fprintf('\n--- For Number of Nodes = %d ---\n', current_nodes);
        CTS_RTS = false; % Disable CTS/RTS for this block

        iteration_collisions = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, current_nodes); % Run simulation for current_nodes
            iteration_collisions(j) = COLLISIONS; % Record total collisions
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        % Store average collisions for the current number of nodes
        without_cts_rts_collisions(i) = sum(iteration_collisions) / num_iterations;
    end

    % --- With CTS/RTS ---
    disp(' ');
    disp('WITH CTS/RTS SIMULATION');
    with_cts_rts_collisions = zeros(1, length(node_range));
    for i = 1:length(node_range)
        current_nodes = node_range(i);
        fprintf('\n--- For Number of Nodes = %d ---\n', current_nodes);
        CTS_RTS = true; % Enable CTS/RTS for this block

        iteration_collisions = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, current_nodes); % Run simulation for current_nodes
            iteration_collisions(j) = COLLISIONS; % Record total collisions
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        with_cts_rts_collisions(i) = sum(iteration_collisions) / num_iterations;
    end

    disp(' ');
    disp('WITHOUT CTS/RTS Collisions:');
    disp(without_cts_rts_collisions);
    disp('WITH CTS/RTS Collisions:');
    disp(with_cts_rts_collisions);

    % Plotting the results
    figure('Position', [100, 100, 800, 500]);
    bar_width = 0.4;
    
    x_no_cts = node_range - bar_width/2;
    x_with_cts = node_range + bar_width/2;

    bar(x_no_cts, without_cts_rts_collisions, bar_width, 'FaceColor', [0.8 0 0], 'EdgeColor', 'k', 'DisplayName', 'Without RTS/CTS'); hold on;
    bar(x_with_cts, with_cts_rts_collisions, bar_width, 'FaceColor', [0 0 0.8], 'EdgeColor', 'k', 'DisplayName', 'With RTS/CTS'); hold off;

    xlabel('Number of Nodes');
    ylabel('Average Number of Collisions');
    title('Collisions vs. Number of Nodes');
    legend('Location', 'best');
    grid on;
    xticks(node_range);
    set(gca, 'FontSize', 10);
    
    disp('Collision Analysis Finished.');
end

function Analyze_Coverage_Area()
    % Analyzes throughput with and without CTS/RTS mechanism across varying coverage areas.
    global PACKETS;
    global NUMBER_OF_NODES;
    global CTS_RTS;
    global LAST_TIME;
    global COVERAGE_RANGE;
    global SIM_TIME;

    disp(' ');
    disp('Starting Coverage Area Analysis: Throughput vs. Coverage Range');
    
    coverage_range_values = 1:15; % Coverage area values to simulate
    fixed_nodes = 10; % Fixed number of nodes for this analysis
    num_iterations = 30; % Number of iterations for averaging each data point

    % --- Without CTS/RTS ---
    disp('WITHOUT CTS/RTS SIMULATION');
    CTS_RTS = false; % Disable CTS/RTS
    without_cts_rts_coverage_throughput = zeros(1, length(coverage_range_values));
    for i = 1:length(coverage_range_values)
        current_coverage_range = coverage_range_values(i);
        fprintf('\n--- For Coverage Area = %d ---\n', current_coverage_range);
        COVERAGE_RANGE = current_coverage_range; % Set the coverage range

        iteration_throughputs = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, fixed_nodes); % Run simulation with fixed_nodes
            
            if LAST_TIME > 0
                iteration_throughputs(j) = PACKETS / LAST_TIME;
            else
                iteration_throughputs(j) = 0;
            end
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        without_cts_rts_coverage_throughput(i) = sum(iteration_throughputs) / num_iterations;
    end

    % --- With CTS/RTS ---
    disp(' ');
    disp('WITH CTS/RTS SIMULATION');
    CTS_RTS = true; % Enable CTS/RTS
    with_cts_rts_coverage_throughput = zeros(1, length(coverage_range_values));
    for i = 1:length(coverage_range_values)
        current_coverage_range = coverage_range_values(i);
        fprintf('\n--- For Coverage Area = %d ---\n', current_coverage_range);
        COVERAGE_RANGE = current_coverage_range; % Set the coverage range

        iteration_throughputs = zeros(1, num_iterations);
        for j = 1:num_iterations
            fprintf('  Iteration %d:\n', j);
            run_simulation(0, fixed_nodes); % Run simulation with fixed_nodes
            
            if LAST_TIME > 0
                iteration_throughputs(j) = PACKETS / LAST_TIME;
            else
                iteration_throughputs(j) = 0;
            end
            disp(['  TOTAL PACKETS TRANSFERED = ', num2str(PACKETS)]);
        end
        with_cts_rts_coverage_throughput(i) = sum(iteration_throughputs) / num_iterations;
    end

    disp(' ');
    disp('WITHOUT CTS/RTS Throughput (Coverage Area):');
    disp(without_cts_rts_coverage_throughput);
    disp('WITH CTS/RTS Throughput (Coverage Area):');
    disp(with_cts_rts_coverage_throughput);

    % Plotting the results
    figure('Position', [100, 100, 800, 500]);
    bar_width = 0.4;
    
    x_no_cts = coverage_range_values - bar_width/2;
    x_with_cts = coverage_range_values + bar_width/2;

    bar(x_no_cts, without_cts_rts_coverage_throughput, bar_width, 'FaceColor', [0.8 0 0], 'EdgeColor', 'k', 'DisplayName', 'Without RTS/CTS'); hold on;
    bar(x_with_cts, with_cts_rts_coverage_throughput, bar_width, 'FaceColor', [0 0 0.8], 'EdgeColor', 'k', 'DisplayName', 'With RTS/CTS'); hold off;

    xlabel('Coverage Area (Number of Nodes per Sensing Region)');
    ylabel('Throughput (Packets/Unit Time)');
    title(sprintf('Throughput Variation by Coverage Area (Fixed %d Nodes)', fixed_nodes));
    legend('Location', 'best');
    grid on;
    xticks(coverage_range_values);
    set(gca, 'FontSize', 10);
    
    disp('Coverage Area Analysis Finished.');
end

% --- Main Execution Block ---
% Uncomment the function calls below to run desired simulations.
% Note: Running all analyses at once might take a considerable amount of time.

% Run a simple simulation with default parameters
Simple_RUN();

% Analyze Throughput with/without CTS/RTS
% Analyze_CTS_RTS();

% Analyze Collisions with/without CTS/RTS
% Analyze_Collisions();

% Analyze Throughput by varying Coverage Area
% Analyze_Coverage_Area();
