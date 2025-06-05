% RecievingNodes.m
% This file contains the class definition for RecievingNodes,
% which simulates the router's behavior in the CSMA/CA simulation.

classdef RecievingNodes
    properties
        processing_time; % Time to process a frame
    end

    methods
        function obj = RecievingNodes(processing_time)
            % Constructor for RecievingNodes
            obj.processing_time = processing_time;
        end

        function [outcome, time_taken] = receive_frame_at_router(obj, sending_node_id, packet_type_from_sender, current_sim_time)
            % This method simulates the router receiving a frame.
            % It returns an outcome struct and the time taken for processing.
            global Data_senders;
            global COLLISIONS;
            global PREVIOUS_COLLISION_TIME;
            global channel_busy;
            global CTS_RTS;
            global SIFS;

            time_taken = obj.processing_time; % Default processing time for a frame
            outcome = struct('type', 'none', 'delay_for_cts', 0, 'delay_for_ack', 0); % Initialize outcome struct

            Data_senders = Data_senders + 1; % Increment active senders counter

            % Collision Detection: If more than one sender is transmitting simultaneously
            if Data_senders > 1
                disp(['COLLISION !! GARBAGE DATA RECEIVED AT ROUTER at ', num2str(current_sim_time)]);
                % Ensure collision is counted only once per time instant
                if PREVIOUS_COLLISION_TIME ~= current_sim_time
                    COLLISIONS = COLLISIONS + 1;
                    PREVIOUS_COLLISION_TIME = current_sim_time;
                end
                outcome.type = 'collision'; % Signal collision
                Data_senders = Data_senders - 1; % Decrement after simulated 'reception'
                return;
            else
                % No collision, process the frame
                if packet_type_from_sender == 0 % RTS frame
                    disp(['ROUTER RECEIVED RTS FROM Node ', num2str(sending_node_id), ' at ', num2str(current_sim_time)]);
                    if CTS_RTS
                        % Router needs to send CTS back
                        channel_busy = true; % Channel becomes busy for CTS transmission
                        outcome.type = 'send_cts';
                        % Delay for CTS transmission (SIFS + router processing time)
                        outcome.delay_for_cts = SIFS + obj.processing_time;
                    else
                        outcome.type = 'rts_received_no_cts'; % RTS received but no CTS mechanism enabled
                    end
                elseif packet_type_from_sender == 1 % Data frame
                    disp(['ROUTER RECEIVED DATA FRAME FROM Node ', num2str(sending_node_id), ' at ', num2str(current_sim_time)]);
                    if CTS_RTS % Even if CTS/RTS is enabled, ACK is sent for data
                        % Router needs to send ACK back
                        channel_busy = true; % Channel becomes busy for ACK transmission
                        outcome.type = 'send_ack';
                        % Delay for ACK transmission (SIFS + router processing time)
                        outcome.delay_for_ack = SIFS + obj.processing_time;
                    else
                        outcome.type = 'data_received_no_ack_sim'; % Data received without explicit ACK signaling
                    end
                else
                    outcome.type = 'unknown_packet_type'; % Should not happen
                end
            end
            Data_senders = Data_senders - 1; % Decrement active senders counter
        end

        function obj = router_release_channel(obj)
            % Router releases the channel after sending CTS/ACK
            global channel_busy;
            channel_busy = false;
        end
    end
end
