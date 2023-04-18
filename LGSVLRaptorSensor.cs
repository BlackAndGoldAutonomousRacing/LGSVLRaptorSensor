/**
 * Copyright (c) 2021-2022 Gaia Platform LLC
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using UnityEngine;
using Simulator.Sensors.UI;
using System.Collections.Generic;
using Simulator.Analysis;
using System.Collections;
using VehiclePhysics;

namespace Simulator.Sensors
{
    [MessageType("iac_msgs/RaceControlHeader")]
    public struct RaceControlHeader
    {
        public Simulator.Bridge.Data.Ros.Time stamp;
        public byte vehicle_number;
        public byte sequence_number;
    }

    [MessageType("iac_msgs/VehicleCommand")]
    public struct VehicleCommand
    {
        public RaceControlHeader header;
        public byte flags;
        public byte position_command;
        public byte track_position;
        public byte laps;
        public ushort laps_fraction;
    }

    [MessageType("iac_msgs/VehicleStatus")]
    public struct VehicleStatus {
        public RaceControlHeader header;
        public byte flags_received;
        public byte flags_met;
        public byte position_command_progress;
        public byte location;
        public byte ct_state;
        public byte sys_state;
    }

    public struct VehicleCommandSvl
    {
        public byte vehicle_number;
        public byte sequence_number;
        public byte flags;
        public byte position_command;
        public byte track_position;
        public byte laps;
        public ushort laps_fraction;
    }

    public struct VehicleStatusSvl
    {
        public byte vehicle_number;
        public byte sequence_number;
        public byte flags_received;
        public byte flags_met;
        public byte position_command_progress;
        public byte location;
        public byte ct_state;
        public byte sys_state;
    }

    public class RaptorDataBridgePlugin : ISensorBridgePlugin
    {
        public void Register(IBridgePlugin plugin)
        {
            if (plugin?.GetBridgeNameAttribute()?.Name == "ROS" || plugin?.GetBridgeNameAttribute()?.Type == "ROS2" ||
            plugin?.GetBridgeNameAttribute()?.Type == "ROS2_Bridge_GAIA"
            )
            {
                plugin.Factory.RegSubscriber(plugin,
                    (VehicleCommand data) => new VehicleCommandSvl() {
                        vehicle_number = data.header.vehicle_number,
                        sequence_number = data.header.sequence_number,
                        flags = data.flags,
                        position_command = data.position_command,
                        track_position = data.track_position,
                        laps = data.laps,
                        laps_fraction = data.laps_fraction,
                    }
                );
                plugin.Factory.RegSubscriber(plugin,
                    (VehicleStatus data) => new VehicleStatusSvl() {
                        vehicle_number = data.header.vehicle_number,
                        sequence_number = data.header.sequence_number,
                        flags_received = data.flags_received,
                        flags_met = data.flags_met,
                        position_command_progress = data.position_command_progress,
                        location = data.location,
                        ct_state = data.ct_state,
                        sys_state = data.sys_state,
                    }
                );
            }
        }
    }

    [SensorType("LGSVL Raptor Sensor", new[] { typeof(VehicleCommand), typeof(VehicleStatus) })]
    public class LGSVLRaptorSensor : SensorBase
    {
        private VehicleVPP VPP;
        private VehicleSMI NonVPP_alt = null;
        private BridgeInstance Bridge;

        private VehicleCommandSvl Command;
        private VehicleStatusSvl Status;

        [SensorParameter]
        public string StatusTopic;

        private void Awake()
        {
            VPP = GetComponentInParent<VehicleVPP>();
            if (VPP == null)
            {
                Debug.LogWarning("Could not find VehicleVPP.");
                NonVPP_alt = GetComponentInParent<VehicleSMI>();
                if (NonVPP_alt == null){
                    Debug.LogWarning("Could not find supported vehicle dynamics model (VPP/VehicleSMI).");
                }
            }
        }

        protected override void Initialize()
        {
        }

        protected override void Deinitialize()
        {

        }

        private void Update()
        {
        }

        private void FixedUpdate()
        {
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            if (bridge?.Plugin?.GetBridgeNameAttribute()?.Type == "ROS2" ||
                bridge?.Plugin?.GetBridgeNameAttribute()?.Type == "ROS2_Bridge_GAIA"
            )
            {
                Bridge = bridge;
                Bridge.AddSubscriber<VehicleCommandSvl>(Topic, data =>
                {
                    if (Time.timeScale == 0f)
                        return;

                    Command = data;
                    if ((data.flags & 1) > 0)
                    {
                        // Purple Flag
                        if (VPP == null){
                            NonVPP_alt.EmergencyStopped = true;
                        } else {
                            VPP.EmergencyStopped = true;
                        }
                    }
                });
                Bridge.AddSubscriber<VehicleStatusSvl>(StatusTopic, data =>
                {
                    if (Time.timeScale == 0f)
                        return;

                    Status = data;
                    if (data.ct_state == 5)
                    {
                        if (VPP == null){
                            NonVPP_alt.StartEngine();
                        } else {
                            VPP.StartEngine();
                        }
                    }

                    if (data.ct_state == 11)
                    {
                        if (VPP == null){
                            NonVPP_alt.StopEngine();
                        } else {
                            VPP.StopEngine();
                        }
                    }

                    if (data.ct_state == 12)
                    {
                        // Purple Flag
                        if (VPP == null){
                            NonVPP_alt.EmergencyStopped = true;
                        } else {
                            VPP.EmergencyStopped = true;
                        }
                    }
                });
            }
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);

            var graphData = new Dictionary<string, object>();

            graphData["flags"] = Command.flags;
            graphData["position_command"] = Command.position_command;
            graphData["track_position"] = Command.track_position;
            graphData["laps"] = Command.laps;
            graphData["laps_fraction"] = Command.laps_fraction;

            graphData["vehicle_number"] = Status.vehicle_number;
            graphData["sequence_number"] = Status.sequence_number;
            graphData["flags_received"] = Status.flags_received;
            graphData["flags_met"] = Status.flags_met;
            graphData["position_command_progress"] = Status.position_command_progress;
            graphData["location"] = Status.location;
            graphData["ct_state"] = Status.ct_state;
            graphData["sys_state"] = Status.sys_state;

            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
        }
    }
}
