using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;
using mapfWin;

namespace mapf;

/// <summary>
/// Runs David Silver's Cooperative A* (CA*).
/// It doesn't find optimal solutions and isn't complete.
/// </summary>
class CooperativeAStar : IStatisticsCsvWriter, ISolver
{
    /// <summary>
    /// The Reservation Table
    /// </summary>
    private HashSet<TimedMove> _reservationTable = [];
    private AgentState[] _allAgentsState;
    /// <summary>
    /// Maps locations (moves) to the time an agent parked there. From that point on they're
    /// blocked.
    /// </summary>
    private readonly Dictionary<Move, int> _parked = [];
    private int[] _pathCosts;
    private SinglePlan[] _paths;
    private int _maxPathCostSoFar;
    private int _expanded;
    private int _generated;
    private int _totalcost;
    private ProblemInstance _problem;
    private Stopwatch _stopwatch;
    private int _initialEstimate;

    private static readonly AgentState.Comparer _comparer = new();

    public CooperativeAStar() {}

    public string GetName() => "CA*";

    public void Clear()
    {
        _reservationTable.Clear();
        _parked.Clear();
        _initialEstimate = 0;
        _maxPathCostSoFar = 0;
        _totalcost = 0;
        _pathCosts = null;
        _paths = null;
    }

    public void ClearStatistics()
    {
        _expanded = 0;
        _generated = 0;
    }

    public int GetExpanded() => _expanded;

    public int GetGenerated() => _generated;

    public long GetMemoryUsed() => Process.GetCurrentProcess().VirtualMemorySize64;

    public void Setup(ProblemInstance instance, Stopwatch stopwatch)
    {
        Clear();
        ClearStatistics();
        _problem = instance;
        _allAgentsState = instance.Agents;
        _pathCosts = new int[_allAgentsState.Length];
        _paths = new SinglePlan[_allAgentsState.Length];
        _stopwatch = stopwatch;
    }

    public Plan GetPlan() => new Plan(_paths.TakeWhile(plan => plan != null));

    public int GetSolutionCost() => _totalcost;

    public void OutputStatisticsHeader(TextWriter output)
    {
        output.Write(ToString() + " expanded" + Run.RESULTS_DELIMITER);
        output.Write(ToString() + " generated" + Run.RESULTS_DELIMITER);
    }

    public override string ToString() => GetName();

    public int GetSolutionDepth() => _totalcost - _initialEstimate;
        
    /// <summary>
    /// Prints statistics of a single run to the given output. 
    /// </summary>
    public void OutputStatistics(TextWriter output)
    {
        Console.WriteLine($"Expanded nodes: {_expanded}");
        Console.WriteLine($"Generated nodes: {_generated}");
        output.Write(_expanded + Run.RESULTS_DELIMITER);
        output.Write(_generated + Run.RESULTS_DELIMITER);
    }

    public int NumStatsColumns => 2;

    public bool Solve()
    {
        foreach (AgentState agent in _allAgentsState)
        {
            if (!singleAgentAStar(agent))
            {
                _totalcost = (int) Constants.SpecialCosts.NO_SOLUTION_COST;
                return false;
            }
        }
        return true;
    }

    public bool AddOneAgent(int index)
    {
        if (!singleAgentAStar(_allAgentsState[index]))
        {
            _totalcost = (int) Constants.SpecialCosts.NO_SOLUTION_COST;
            return false;
        }
        return true;
    }

    private bool singleAgentAStar(AgentState agent)
    {
        AgentState.EquivalenceOverDifferentTimes = false;
        PriorityQueue<AgentState> openList = new( _comparer );
        HashSet<AgentState> closedList = [];
        agent.H = _problem.GetSingleAgentOptimalCost(agent);
        openList.Push(agent);
        AgentState node;
        _initialEstimate += agent.H;
        TimedMove queryTimedMove = new();

        while (openList.Count > 0)
        {
            if (_stopwatch.ElapsedMilliseconds > Constants.MAX_TIME)
            {
                return false;
            }
            node = openList.Top;
            openList.Remove(node);
            if (node.H == 0)
            {
                bool valid = true;
                for (int i = node.LastMove.Time ; i <= _maxPathCostSoFar; i++)
                {
                    queryTimedMove.Setup(node.LastMove.X, node.LastMove.Y, Direction.NO_DIRECTION, i);
                    if (_reservationTable.Contains(queryTimedMove))
                        valid = false;
                }
                if (valid)
                {
                    _paths[agent.Agent.agentNum] = new SinglePlan(node);
                    reservePath(node);
                    _totalcost += node.LastMove.Time;
                    _parked.Add(new Move(node.LastMove.X, node.LastMove.Y, Direction.NO_DIRECTION), node.LastMove.Time);
                    return true;
                }
            }
            expandNode(node, openList, closedList);
            _expanded++;
        }
        return false;
    }

    private void reservePath(AgentState end)
    {
        AgentState node = end;
        while (node != null)
        {
            _reservationTable.Add(new TimedMove(node.LastMove));
            _pathCosts[node.Agent.agentNum]++;
            node = node.Prev;
        }
        if (_pathCosts[end.Agent.agentNum] > _maxPathCostSoFar)
            _maxPathCostSoFar = _pathCosts[end.Agent.agentNum];
    }

    private void expandNode(AgentState node, PriorityQueue<AgentState> openList, HashSet<AgentState> closedList)
    {
        foreach (TimedMove move in node.LastMove.GetNextMoves())
        {
            if (isValidMove(move))
            {
                AgentState child = new(node)
                {
                    Prev = node
                };
                child.MoveTo(move);
                if (closedList.Contains(child) == false)
                {
                    closedList.Add(child);
                    child.H = _problem.GetSingleAgentOptimalCost(child);
                    openList.Push(child);
                    _generated++;
                }
            }
        }

    }

    private bool isValidMove(TimedMove move)
    {
        if (_problem.IsValid(move) == false)
            return false;
        if (move.IsColliding(_reservationTable))
            return false;
        Move queryMove = new(move.X, move.Y, Direction.NO_DIRECTION);
        if (_parked.ContainsKey(queryMove) && _parked[queryMove] <= move.Time)
            return false;
        return true;
    }
}
