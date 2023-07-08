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
    [MessageType("iac_msgs/RaptorMockupCommand")]
    public struct RaptorMockupCommand
    {
        public Simulator.Bridge.Data.Ros.Time stamp;
        public byte vehicle_number;
        //public byte sequence_number;
        public bool engine_ignition_command;
        public bool emergency_stop_command;
    }

    public struct VehicleCommandSvl
    {
        public byte vehicle_number;
        public bool engine_ignition_command;
        public bool emergency_stop_command;
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
                    (RaptorMockupCommand data) => new VehicleCommandSvl() {
                        vehicle_number = data.vehicle_number,
                        engine_ignition_command = data.engine_ignition_command,
                        emergency_stop_command = data.emergency_stop_command,
                    }
                );
            }
        }
    }

    [SensorType("LGSVL Raptor Sensor", new[] { typeof(RaptorMockupCommand) })]
    public class LGSVLRaptorSensor : SensorBase
    {
        private VehicleVPP VPP;
        private VehicleSMI NonVPP_alt = null;
        private BridgeInstance Bridge;

        private VehicleCommandSvl Command;

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
                    if (data.emergency_stop_command)
                    {
                        // Purple Flag
                        if (VPP == null){
                            NonVPP_alt.EmergencyStopped = true;
                        } else {
                            VPP.EmergencyStopped = true;
                        }
                    }

                    if (data.engine_ignition_command)
                    {
                        if (VPP == null){
                            NonVPP_alt.StartEngine();
                        } else {
                            VPP.StartEngine();
                        }
                    } else {
                        if (VPP == null){
                            NonVPP_alt.StopEngine();
                        } else {
                            VPP.StopEngine();
                        }
                    }
                });
            }
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);

            var graphData = new Dictionary<string, object>();

            graphData["vehicle_number"] = Command.vehicle_number;
            graphData["engine_ignition_command"] = Command.engine_ignition_command;
            graphData["emergency_stop_command"] = Command.emergency_stop_command;

            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
        }
    }
}
