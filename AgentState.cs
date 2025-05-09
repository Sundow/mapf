using System;
using System.Collections.Generic;

namespace mapf;

public class AgentState
{
    /// <summary>
    /// Only used when AgentState objects are put in the open list priority queue - mainly in AStarForSingleAgent, I think.
    /// </summary>
    public int H { get; set; }
    public Agent Agent { get; private set; }
    /// <summary>
    /// At goal
    /// </summary>
    public int ArrivalTime { get; private set; }
    /// <summary>
    /// The last move's time is the agent's G
    /// </summary>
    public TimedMove LastMove { get; private set; }
    /// <summary>
    /// Only used by AStarForSingleAgent, which should itself be deleted.
    /// </summary>
    public AgentState Prev { get; set; }
    /// <summary>
    /// For CBS this must be set to false.
    /// </summary>
    public static bool EquivalenceOverDifferentTimes = true;

    public AgentState(int pos_X, int pos_Y, Agent agent)
    {
        LastMove = new TimedMove(pos_X, pos_Y, Direction.NO_DIRECTION, 0);
        Agent = agent;
    }

    public AgentState(int startX, int startY, int goalX, int goalY, int agentId)
        : this(startX, startY, new Agent(goalX, goalY, agentId))
    { }

    public AgentState(AgentState copy)
    {
        Agent = copy.Agent;
        H = copy.H;
        ArrivalTime = copy.ArrivalTime;
        LastMove = copy.LastMove; //new TimedMove(copy.lastMove); // Can we just do lastMove = copy.lastMove? I think we can now, since MoveTo replaces the move
        //prev = copy;
        G = copy.G;
    }

    /// <summary>
    /// Only used by EnumeratedPDB - check if can be removed
    /// </summary>
    public void SwapCurrentWithGoal()
    {
        int nTemp = LastMove.X;
        LastMove.X = Agent.Goal.X;
        Agent.Goal.X = nTemp;
        nTemp = LastMove.Y;
        LastMove.Y = Agent.Goal.Y;
        Agent.Goal.Y = nTemp;
    }

    /// <summary>
    /// Updates the agent's last move with the given move and sets arrivalTime (at goal) if necessary.
    /// </summary>
    public void MoveTo(TimedMove move)
    {
        LastMove = move;

        bool isWait = move.Direction == Direction.Wait;
        bool atGoal = AtGoal();

        // If performed a non WAIT move and reached the agent's goal - store the arrival time
        if (atGoal && (isWait == false))
            ArrivalTime = move.Time;

        if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
        {
            if (AtGoal())
                G = ArrivalTime;
            else
                G = LastMove.Time;
        }
        else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
        {
            if ((atGoal && isWait) == false)
                G += 1;
        }
    }

    /// <summary>
    /// Checks if the agent is at its goal location
    /// </summary>
    /// <returns>True if the agent has reached its goal location</returns>
    public bool AtGoal() => Agent.Goal.Equals(LastMove); // Comparing Move to TimedMove is allowed, the reverse isn't.

    public int G { get; private set; }

    /// <summary>
    /// When equivalence over different times is necessary,
    /// checks agent and last position only,
    /// ignoring data that would make this state different to other equivalent states:
    /// It doesn't matter from which direction the agent got to its current location.
    /// It's also necessary to ignore the agents' move time - we want the same positions
    /// in any time to be equivalent.
    /// </summary>
    /// <param name="obj"></param>
    /// <returns></returns>
    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        AgentState that = (AgentState)obj;

        if (AgentState.EquivalenceOverDifferentTimes)
        {
            return Agent.Equals(that.Agent) &&
                    LastMove.X == that.LastMove.X &&
                    LastMove.Y == that.LastMove.Y; // Ignoring the time and the direction
        }
        else
        {
            return Agent.Equals(that.Agent) &&
                    LastMove.X == that.LastMove.X &&
                    LastMove.Y == that.LastMove.Y &&
                    LastMove.Time == that.LastMove.Time; // Ignoring the direction
        }
    }

    /// <summary>
    /// When equivalence over different times is necessary,
    /// uses agent and last position only, ignoring direction and time.
    /// </summary>
    /// <returns></returns>
    public override int GetHashCode()
    {
        unchecked
        {
            if (AgentState.EquivalenceOverDifferentTimes)
                return 3 * Agent.GetHashCode() + 5 * LastMove.X + 7 * LastMove.Y;
            else
                return 3 * Agent.GetHashCode() + 5 * LastMove.GetHashCode();
        }
    }

    public Move GetMove() => LastMove;

    public override string ToString() => $"step-{LastMove.Time} position {LastMove}";

    public class Comparer : IComparer<AgentState>
    {
        public int Compare(AgentState x, AgentState y)
        {
            if (x.H + x.LastMove.Time < y.H + y.LastMove.Time)
                return -1;
            if (x.H + x.LastMove.Time > y.H + y.LastMove.Time)
                return 1;

            // TODO: Prefer goal nodes.

            // Prefer larger g:
            if (x.LastMove.Time < y.LastMove.Time)
                return 1;
            if (x.LastMove.Time > y.LastMove.Time)
                return -1;
            return 0;
        }
    }
}
