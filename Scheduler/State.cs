﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace HSFScheduler
{
    public class State
    {
        /** The previous state, upon which this state is based */
        public State _previous { get; private set; }

        /** The start of the event associated with this State */
        public double _eventStart{get; set;}

        /** The start of the task associated with this State */
        public double _taskStart { get; set; }

        /** The end of the task associated with this State */
        public double _taskEnd { get; set; }

        /** The end of the event associated with this State */
        public double _eventEnd { get; set; }

        /** The Dictionary of integer Profiles. */
        private Dictionary<StateVarKey, HSFProfile<int>> idata { get; }

        /** The Dictionary of double precision Profiles. */
        private Dictionary<StateVarKey, HSFProfile<double>> ddata { get; }

        /** The Dictionary of floating point value Profiles. */
        private Dictionary<StateVarKey, HSFProfile<float>> fdata { get; }

        /** The Dictionary of boolean Profiles. */
        private Dictionary<StateVarKey, HSFProfile<bool>> bdata { get; }

        /** The Dictionary of Matrix Profiles. */
        private Dictionary<StateVarKey, HSFProfile<Matrix>> mdata { get; }

        /** The Dictionary of Quaternion Profiles. */
        private Dictionary<StateVarKey, HSFProfile<Quat>> qdata { get; }


        /**
         * Creates an initial State
         */
        State()
        {
            _previous = null;
            _eventStart = 0;
            _eventEnd = 0;
            _taskEnd = 0;
            _taskStart = 0;
        }

        /**
         * Copy constructor for exact state copies
         */
        State(State initialStateToCopy)
        {
            _previous = initialStateToCopy._previous;
            _eventStart = initialStateToCopy._eventStart;
            _taskStart = initialStateToCopy._taskStart;
            _taskEnd = initialStateToCopy._taskEnd;
            _eventEnd = initialStateToCopy._eventEnd;
            idata = initialStateToCopy.idata;
            ddata = initialStateToCopy.ddata;
            fdata = initialStateToCopy.fdata;
            bdata = initialStateToCopy.bdata;
            mdata = initialStateToCopy.mdata;
            qdata = initialStateToCopy.qdata;
        }

        /**
         * Creates a new State based on a previous State and a new Task start time
         */
        State(State previous, double newTaskStart)
        {
            _previous = previous;
            _eventStart = previous._eventEnd; // start from end of previous State
            _taskStart = newTaskStart;
            _taskEnd = newTaskStart;
            _eventEnd = newTaskStart;
        }


        //took out setters and getters becuase they're public fields

        /** TODO: figure out if this can all be done with dictionary stuff
         * Gets the last int value set for the given state variable key in the state. If no value is found
         * it checks the previous state, continuing all the way to the initial state.
         * @param key The integer state variable key that is being looked up.
         * @return A pair containing the last time the variable was set, and the integer value.
         */
         KeyValuePair<double, int> getLastValue(StateVarKey key) {
            HSFProfile<int> valueOut;
		    if(idata.Count != 0) { // Are there any Profiles in there?
                if (idata.TryGetValue(key, out valueOut)) //see if our key is in there
                    return valueOut.Last(); //found it, return it TODO: return last value or pair?
		    }
            return _previous.getLastValue(key); //either no profiles or none that match our keys, try finding it in the previous one
        }
        /** 
         * Gets the integer value of the state at a certain time. If the exact time is not found, the data is
         * assumed to be on a zero-order hold, and the last value set is found.
         * @param key The integer state variable key that is being looked up.
         * @param time The time the value is looked up at.
         * @return A pair containing the last time the variable was set, and the integer value.
         */
        KeyValuePair<double, int> getValueAtTime(StateVarKey key, double time){
            HSFProfile<int> valueOut;
            if (idata.Count != 0)  { // Are there any Profiles in there?
                if (idata.TryGetValue(key, out valueOut)) //see if our key is in there
                    return valueOut.DataAtTime(time);
            }
		    return  _previous.getValueAtTime(key, time); //either no profiles or none that match our keys, try finding it in the previous one
        }

        /** 
         * Returns the integer Profile matching the key given. If no Profile is found, it goes back one Event
         * and checks again until it gets to the initial state.
         * @param key The integer state variable key that is being looked up.
         * @return The Profile saved in the state.
         */
        HSFProfile<int> getProfile(StateVarKey key){
            HSFProfile<int> valueOut;
		    if(idata.Count != 0)  { // Are there any Profiles in there?
                if (idata.TryGetValue(key, out valueOut)) //see if our key is in there
                    return valueOut;
            }
		    return _previous.getProfile(key); // This isn't the right profile, go back one and try it out!
	    }

        /** TODO: make sure valueOut is a good replacement for iterator.second
         * Returns the integer Profile for this state and all previous states merged into one Profile
         * @param key The integer state variable key that is being looked up.
         * @return The full Profile
         */
        HSFProfile<int> getFullProfile(StateVarKey key){
            HSFProfile<int> valueOut = new HSFProfile<int>();
		    if(idata.Count != 0)  { // Are there any Profiles in there?
                if (idata.TryGetValue(key, out valueOut)) { //see if our key is in there
                    if (_previous != null) // Check whether we are at the first state
					    return HSFProfile<int>.MergeProfiles(valueOut, _previous.getFullProfile(key));
				    return valueOut;				
			    }
            }
		    if(_previous != null)
			    return _previous.getFullProfile(key); // If no data, return profile from previous states
		    return valueOut; //return empty profile
	   }

        /** 
         * Sets the integer Profile in the state with its matching key. If a Profile is found already under
         * that key, this will overwrite the old Profile.
         * @param key The integer state variable key that is being set.\
         * @param profIn The integer Profile being saved.
         */
        void setProfile(StateVarKey key, HSFProfile<int> profIn){
            HSFProfile<int> valueOut;
            if (!idata.TryGetValue(key, out valueOut)) // If there's no Profile matching that key, insert a new one.
                idata.Add(key, profIn); 
		    else { // Otherwise, erase whatever is there, and insert a new one.
			    idata.Remove(key);
			    idata.Add(key, profIn); 
		    }
        }

        /** 
	     * Adds a integer Profile value pair to the state with the given key. If no Profile exists, a new Profile is created
	     * with the corresponding key. If a Profile exists with that key, the pair is appended onto the end of the Profile. 
	     * Ensure that the Profile is still time ordered if this is the case.
	     * @param key The key corresponding to the state variable.
	     * @param pairIn The pair to be added to the integer Profile.
	     */
        void addValue(StateVarKey key, KeyValuePair<double, int> pairIn){
            HSFProfile<int> valueIn = new HSFProfile<int>(pairIn);
            HSFProfile<int> valueOut;
            if (!idata.TryGetValue(key, out valueOut)) // If there's no Profile matching that key, insert a new one.
                idata.Add(key, valueIn); 
		    else // Otherwise, add this data point to the existing Profile.
			    valueOut.Add(pairIn); //TODO: make sure this is ok. was formally iterator.second.data
	    }

        /** 
         * Adds a integer Profile to the state with the given key. If no Profile exists, a new Profile is created
         * with the corresponding key. If a Profile exists with that key, the Profile is appended onto the end of the Profile. 
         * @param key The key corresponding to the state variable.
         * @param profIn The Profile to be added to the integer Profile.
         */
        void addValue(StateVarKey key, HSFProfile<int> profIn){
		    HSFProfile<int> valueOut;
            if (!idata.TryGetValue(key, out valueOut)) // If there's no Profile matching that key, insert a new one.
			    idata.Add(key, profIn); 
		    else // Otherwise, add this data point to the existing Profile.
			    valueOut.Add(profIn);
	    }
}
}