﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HSFSystem;

namespace HSFScheduler
{
    /// <summary>
    /// A class used to calculate and store all access windows from all systems to all task targets
    /// Also used to retrieve all accesses currently available to a system at a given simulation time
    /// This class replaces the geomAccess class from Horizon 2.3
    /// </summary>
    public class Access
    {
        public Asset Asset { get; private set; }
        public Task Task { get; private set; }
        public double AccessStart { get; private set; }
        public double AccessEnd { get; private set; }

        public Access(Asset asset, Task task)
        {
            Asset = asset;
            Task = task;
        }

        public static Stack<Access> getCurrentAccessesForAsset(Stack<Access> accesses, Asset asset, double currentTime)
        {
            Stack<Access> allAccesses = Access.getCurrentAccesses(accesses, currentTime);
            return (Stack<Access>) allAccesses.Where(item => item.Asset == asset);
        }

        public static Stack<Access> getCurrentAccesses(Stack<Access> accesses, double currentTime)
        {
              return (Stack<Access>)accesses.Where(item => (item.AccessStart <= currentTime && item.AccessEnd >= currentTime));
        }

        /// <summary>
        /// PreGenerate all Accesses from all Assets to all Tasks based on a simple Line of Sight model.  Access is based on Line of Sight, a spherical Earth and the current ECI
        /// position of the Asset and the Task.Target.DynamicState
        /// </summary>
        /// <param name="system">The system underconsideration</param>
        /// <param name="tasks"></param>
        /// <param name="startTime"></param>
        /// <param name="endTime"></param>
        /// <param name="stepTime"></param>
        public static Stack<Access> pregenerateAccessesByAsset(SystemClass system, Stack<Task> tasks, double startTime, double endTime, double stepTime)
        {
            Stack<Access> accessesByAsset = new Stack<Access>();
            // For all assets...
            foreach (Asset asset in system.Assets)
            {
                // ...for all tasks...
                foreach (Task task in tasks)
                {
                    // ...for all time....    
                    for (double accessTime = startTime; accessTime <= endTime; accessTime += stepTime)
                    {
                        // create a new access, or extend the access endTime if this is an update to an existing access
                        bool hasAccess = Utilities.GeometryUtilities.hasLOS(asset.AssetDynamicState.PositionECI(accessTime), task.Target.DynamicState.PositionECI(accessTime));
                        if (hasAccess)
                        {
                            bool isNewAccess = (accessTime - accessesByAsset.Peek().AccessEnd) >= stepTime;
                            if (isNewAccess)
                            {
                                Access newAccess = new Access(asset, task);
                                newAccess.AccessStart = accessTime;
                                newAccess.AccessEnd = accessTime;
                                accessesByAsset.Push(newAccess);
                            }
                            else  // extend the access
                                accessesByAsset.Peek().AccessEnd = accessTime;
                        }
                    }
                }
            }
            return accessesByAsset;
        }
    }
}
