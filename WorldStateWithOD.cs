using System.Collections.Generic;
using System.Linq;

namespace mapf;

/// <summary>
/// This class represents a state in the A* search with operator decomposition,
/// as proposed by Scott Standley's AAAI paper in 2010.
/// More specifically, states can represent a partial move, in which only some of the agents have moved
/// and the other have not yet moved in this turn. 
/// </summary>
public class WorldStateWithOD : WorldState
{
    /// <summary>
    /// Marks the index of the agent that will move next. 
    /// All agents with index less than agentTurn are assumed to have already chosen their move for this time step,
    /// while agents with higher index have not chosen their move yet.
    /// </summary>
    public int AgentTurn { get; set; }

    public WorldStateWithOD(AgentState[] states, int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
        : base(states, minDepth, minCost, mddNode)
    {
        AgentTurn = 0;
    }

    public WorldStateWithOD(WorldStateWithOD cpy) : base(cpy)
    {
        AgentTurn = cpy.AgentTurn;
    }

    /// <summary>
    /// Used for PDB stuff only
    /// </summary>
    /// <param name="states"></param>
    /// <param name="relevantAgents"></param>
    public WorldStateWithOD(AgentState[] states, List<uint> relevantAgents) : base(states, relevantAgents)
    {
        AgentTurn = 0;
    }

    public override (ProblemInstance, ISet<CbsConstraint>) ToProblemInstance(ProblemInstance initial)
    {
        WorldState state = this;
        if (AgentTurn != 0)
        {
            // CBS doesn't handle partially expanded nodes well.
            // Use the last fully expanded node and add the additional moves as must conds:
            state = PrevStep; // Points to the last fully expanded node.
        }

        ProblemInstance subproblem = initial.Subproblem(state.AllAgentsState); // Can't use base's method because we're operating on a different object
        HashSet<CbsConstraint> positiveConstraints = [];
        if (AgentTurn != 0)
        {
            for (int i = 0; i < AgentTurn; ++i)
            {
                positiveConstraints.Add(new CbsConstraint(AllAgentsState[i].Agent.agentNum, AllAgentsState[i].LastMove));
            }
        }

        return (subproblem, positiveConstraints);
    }

    /// <summary>
    /// Set the optimal solution of this node as a problem instance.
    /// </summary>
    /// <param name="solution"></param>
    public override void SetSolution(SinglePlan[] solution)
    {
        if (AgentTurn == 0)
            singlePlans = SinglePlan.GetSinglePlans(this);
        else
            singlePlans = SinglePlan.GetSinglePlans(PrevStep);
        // ToProblemInstance gives the last proper state as the problem to solve,
        // with must constraints to make the solution go through the steps already
        // taken from there.

        for (int i = 0; i < solution.Length; ++i)
            singlePlans[i].ContinueWith(solution[i]);
    }

    public override string ToString()
    {
        string ans = base.ToString();
        if (AgentTurn == 0)
            return ans;
        else
            return $"Partial node {ans}, agent turn: {AgentTurn}";
    }

    /// <summary>
    /// Returns a hash value for the given state (used in Hash based data structures).
    /// </summary>
    /// <returns></returns>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = Constants.PRIMES_FOR_HASHING[0];
            hash = hash * Constants.PRIMES_FOR_HASHING[1] + base.GetHashCode();
            hash = hash * Constants.PRIMES_FOR_HASHING[2] + AgentTurn;
            return hash;
        }
    }

    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        var that = (WorldStateWithOD)obj;
        if (that.AgentTurn != AgentTurn)
            // It's tempting to think that this check is enough to allow equivalence over different times,
            // because it differentiates between a state where all agents have moved and its
            // child where the first agent WAITed, allowing the child
            // to be generated because it isn't a hit in the closed list.
            // But it isn't enough.
            // This may a partially generated node,
            // and we may have already gotten to this specific set of agent positions,
            // but from a different set of locations, so the allowed moves of the remaining agents
            // that haven't already moved would be different.
            return false;

        if (AgentTurn == 0) // All agents have moved, safe to ignore direction information.
            return base.Equals(obj);

        if (AllAgentsState.Length != that.AllAgentsState.Length)
            return false;

        // Comparing the agent states:
        for (int i = 0; i < AllAgentsState.Length; ++i)
        {
            if (AllAgentsState[i].Equals(that.AllAgentsState[i]) == false)
                return false;
            if (i < AgentTurn) // Agent has already moved in this step
            {
                bool mightCollideLater = false;
                for (int j = AgentTurn; j < AllAgentsState.Length; j++)
                {
                    if (AllAgentsState[i].LastMove.X == AllAgentsState[j].LastMove.X &&
                        AllAgentsState[i].LastMove.Y == AllAgentsState[j].LastMove.Y) // Can't just remove the direction and use IsColliding since the moves' time is different, so they'll never collide
                    {
                        mightCollideLater = true;
                        break;
                    }
                }

                if (mightCollideLater == true) // Then check the direction too
                {
                    if (AllAgentsState[i].LastMove.Direction != Direction.NO_DIRECTION &&
                            that.AllAgentsState[i].LastMove.Direction != Direction.NO_DIRECTION &&
                            AllAgentsState[i].LastMove.Direction != that.AllAgentsState[i].LastMove.Direction) // Can't just use allAgentsState[i].lastMove.Equals(that.allAgentsState[i].lastMove) because TimedMoves don't ignore the time.
                        return false;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Used when WorldStateWithOD objects are put in the open list priority queue.
    /// All other things being equal, prefers nodes where more agents have moved.
    /// G is already preferred, but this helps when the last move was a WAIT at the
    /// goal, which doesn't increment G.
    /// </summary>
    /// <param name="other"></param>
    /// <returns></returns>
    public override int CompareTo(WorldState other)
    {
        int res = base.CompareTo(other);
        if (res != 0)
            return res;

        if (other is not WorldStateWithOD that)
            return 0;

        // Further tie-breaking
        // Prefer more fully generated nodes:
        // For the same F, they're probably closer to the goal.
        // The goal isn't necessarily a fully expanded node.
        // A*+OD may finish when all agents reached their goal even if it isn't a fully expanded state, and that's a nice feature!
        // So we prefer more fully generated nodes just because it gives a more DFS-like behavior
        // on the heuristic's fast path to the goal.
        if (AgentTurn == 0 && that.AgentTurn != 0)
            return -1;
        if (that.AgentTurn == 0 && AgentTurn != 0)
            return 1;
        return that.AgentTurn.CompareTo(AgentTurn); // Notice the order inversion - bigger is better.
    }

    /// <summary>
    /// Counts for last agent to move only, the counts from the previous agents to move are accumulated from the parent node.
    /// </summary>
    /// <param name="conflictAvoidance"></param>
    /// <returns></returns>
    public override void IncrementConflictCounts(ConflictAvoidanceTable conflictAvoidance)
    {
        int lastAgentToMove = AgentTurn - 1;
        if (AgentTurn == 0)
            lastAgentToMove = AllAgentsState.Length - 1;

        AllAgentsState[lastAgentToMove].LastMove.IncrementConflictCounts(conflictAvoidance,
                                                                        ConflictCounts, ConflictTimes);
        _primaryTieBreaker = ConflictCounts.Sum(pair => pair.Value);
    }
}
