using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using ExtensionMethods;

namespace mapf;

/// <summary>
/// Finds the solution with the least number of conflicts, given a set of MDDs
/// </summary>
class A_Star_MDDs : IConflictReporting
{
    private MDD[] _problem;
    private readonly Dictionary<A_Star_MDDs_Node, A_Star_MDDs_Node> _closedList = [];
    private readonly Stopwatch _stopwatch;

    private readonly SortedSet<A_Star_MDDs_Node> _openList = new( new A_Star_MDDs_Node.Comparer() );

    public int Expanded { get; private set; } = 0;
    public int Generated { get; private set; } = 0;
    public int ConflictCount { get; private set; } = 0;
    private readonly ConflictAvoidanceTable _cat;

    public A_Star_MDDs(MDD[] problem, Stopwatch stopwatch, ConflictAvoidanceTable CAT)
    {
        A_Star_MDDs_Node root;
        _problem = problem;
        _stopwatch = stopwatch;
        _cat = CAT;
        MDDNode[] sRoot = new MDDNode[problem.Length];
        for (int i = 0; i < problem.Length; i++)
        {
            sRoot[i] = problem[i].levels[0].First.Value;
        }
        root = new A_Star_MDDs_Node(sRoot, null);
        _openList.Add(root);
        _closedList.Add(root, root); // There will never be a hit. This is only done for consistancy
    }
       
    public SinglePlan[] Solve()
    {
        A_Star_MDDs_Node currentNode;

        while (_openList.Count > 0)
        {
            if (_stopwatch.ElapsedMilliseconds > Constants.MAX_TIME)
            {
                return null;
            }

            currentNode = _openList.Min;
            _openList.Remove(currentNode);

            // Check if node is the goal
            if (GoalTest(currentNode))
            {
                ConflictCount = currentNode.ConflictCount;
                _conflictCounts = currentNode.ConflictCounts;
                _conflictTimes = currentNode.ConflictTimes;
                return GetAnswer(currentNode);
            }

            // Expand
            Expanded++;  // TODO: don't count re-expansions as expansions?
            Expand(currentNode);
            //expander.Setup(currentNode);
            //Expand(expander);  // TODO: the expander just generates all children. EPEA* its ass!!
        }
        return null;
    }

    public void Expand(A_Star_MDDs_Node node)
    {
        if (node.IsAlreadyExpanded() == false)
        {
            node.calcSingleAgentDeltaConflictCounts(_cat);
            node.AlreadyExpanded = true;
            node.TargetDeltaConflictCount = 0;
            node.RemainingDeltaConflictCount = node.TargetDeltaConflictCount; // Just for the following hasChildrenForCurrentDeltaConflictCount call.
            while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount() == false) // DeltaConflictCount==0 may not be possible if all agents have obstacles between their location and the goal
            {
                node.TargetDeltaConflictCount++;
                node.RemainingDeltaConflictCount = node.TargetDeltaConflictCount;
            }
            if (node.hasMoreChildren() == false) // Node has no possible children at all
            {
                node.ClearExpansionData();
                return;
            }
        }

        var intermediateNodes = new List<A_Star_MDDs_Node>() { node };

        for (int mddIndex = 0; mddIndex < _problem.Length && intermediateNodes.Count != 0; ++mddIndex)
        {
            if (_stopwatch.ElapsedMilliseconds > Constants.MAX_TIME)
                return;

            intermediateNodes = ExpandOneAgent(intermediateNodes, mddIndex);
        }

        var finalGeneratedNodes = intermediateNodes;

        foreach (var child in finalGeneratedNodes)
        {
            child.ConflictCount = node.ConflictCount + node.TargetDeltaConflictCount;

            // Accumulating the conflicts count from parent to child
            // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
            child.ConflictCounts = new Dictionary<int, int>(child.Prev.ConflictCounts);
            child.ConflictTimes = [];
            foreach (var kvp in child.Prev.ConflictTimes)
                child.ConflictTimes[kvp.Key] = [.. kvp.Value];
            child.IncrementConflictCounts(_cat);  // We're counting conflicts along the entire path, so the parent's conflicts count
                                                    // is added to the child's.

            bool was_closed = _closedList.ContainsKey(child);
            if (was_closed)
            {
                A_Star_MDDs_Node inClosedList = _closedList[child];

                if (inClosedList.ConflictCount > child.ConflictCount)
                {
                    _closedList.Remove(inClosedList);
                    _openList.Remove(inClosedList);
                    was_closed = false;
                }
            }
            if (!was_closed)
            {
                _openList.Add(child);
                _closedList.Add(child, child);
                Generated++;
            }
        }

        // Prepare the node for the next partial expansion:
        if (node.IsAlreadyExpanded() == false)
        {
            // Node was cleared during expansion.
            // It's unnecessary and unsafe to continue to prepare it for the next partial expansion.
            return;
        }

        node.TargetDeltaConflictCount++; // This delta F was exhausted
        node.RemainingDeltaConflictCount = node.TargetDeltaConflictCount;

        while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount() == false)
        {
            node.TargetDeltaConflictCount++;
            node.RemainingDeltaConflictCount = node.TargetDeltaConflictCount; // Just for the following hasChildrenForCurrentDeltaF call.
        }

        if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount())
        {
            // Re-insert node into open list
            _openList.Add(node);
        }
        else
            node.ClearExpansionData();
    }

    protected List<A_Star_MDDs_Node> ExpandOneAgent(List<A_Star_MDDs_Node> intermediateNodes, int mddIndex)
    {
        List<A_Star_MDDs_Node> generated = [];

        // Expand the mdd node
        foreach (A_Star_MDDs_Node node in intermediateNodes)
        {
            // Try all the children of this MDD node
            foreach ((int childIndex, MDDNode childMddNode) in node.AllSteps[mddIndex].children.Enumerate())
            {
                if (node.CurrentMoves != null && childMddNode.move.IsColliding(node.CurrentMoves))  // Can happen. We only prune partially, we don't build the full k-agent MDD.
                    continue;

                A_Star_MDDs_Node childNode = new(node, mddIndex != node.AllSteps.Length - 1);
                childNode.AllSteps[mddIndex] = childMddNode;

                // Update target conflict count and prune nodes that can't get to the target conflict count
                childNode.UpdateRemainingDeltaConflictCount(mddIndex, childIndex);
                if (childNode.RemainingDeltaConflictCount == ushort.MaxValue || // Last move was bad - not sure this can happen here
                    (childNode.hasChildrenForCurrentDeltaConflictCount(mddIndex + 1) == false))  // No children that can reach the target
                    continue;

                if (mddIndex < node.AllSteps.Length - 1) // More MDD nodes need to choose a child
                    childNode.CurrentMoves.Add(childMddNode.move);
                else // Moved the last agent
                    childNode.CurrentMoves = null; // To reduce memory load and lookup times

                // Set the node's prev to its real parent, skipping over the intermediate nodes.
                if (mddIndex != 0)
                    childNode.Prev = node.Prev;
                else
                    childNode.Prev = node;

                generated.Add(childNode);
            }
        }
            
        return generated;
    }

    private Dictionary<int, int> _conflictCounts;
    private Dictionary<int, List<int>> _conflictTimes;

    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to the number of conflicts with their path the solution has</returns>
    public Dictionary<int, int> GetExternalConflictCounts() => _conflictCounts;

    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to a list of times the solution has a conflict with theirs</returns>
    public Dictionary<int, List<int>> GetConflictTimes() => _conflictTimes;
    
    public void Expand(A_Star_MDDs_Expander currentNode)
    {
        while (true)
        {
            A_Star_MDDs_Node child = currentNode.GetNextChild();
            if (child == null)
                break;

            if (IsLegalMove(child))
            {
                child.ConflictCount = child.Prev.ConflictCount;
                child.UpdateConflicts(_cat);

                bool was_closed = _closedList.ContainsKey(child);
                if (was_closed)
                {
                    A_Star_MDDs_Node inClosedList = _closedList[child];

                    if (inClosedList.ConflictCount > child.ConflictCount)
                    {
                        _closedList.Remove(inClosedList);
                        _openList.Remove(inClosedList);
                        was_closed = false;
                    }
                }
                if (!was_closed)
                {
                    _closedList.Add(child, child);
                    Generated++;
                }
            }
        }
    }

    private bool GoalTest(A_Star_MDDs_Node toCheck)
    {
        if (toCheck.GetDepth() == _problem[0].levels.Length - 1)
            return true;
        return false;
    }

    private SinglePlan[] GetAnswer(A_Star_MDDs_Node finish)
    {
        // TODO: Move the construction of the SinglePlans to a static method in SinglePlan
        List<Move>[] routes = new List<Move>[_problem.Length];
        for (int i = 0; i < routes.Length; i++)
            routes[i] = [];

        A_Star_MDDs_Node current = finish;
        while (current != null)
        {
            for (int i = 0; i < _problem.Length; i++)
            {
                routes[i].Add(new Move(current.AllSteps[i].move));
            }
            current = current.Prev;
        }

        SinglePlan[] ans = new SinglePlan[_problem.Length];
        for (int i = 0; i < ans.Length; i++)
        {
            routes[i].Reverse();
            ans[i] = new SinglePlan(routes[i], i);
        }
        return ans;
    }
        
    private bool CheckIfLegal(MDDNode to1, MDDNode to2) => to1.move.IsColliding(to2.move) == false;
        
    private bool IsLegalMove(A_Star_MDDs_Node to)
    {
        if (to == null)
            return false;
        if (to.Prev == null)
            return true;
        for (int i = 0; i < _problem.Length; i++)
        {
            for (int j = i+1; j < to.AllSteps.Length; j++)
            {
                if (CheckIfLegal(to.AllSteps[i], to.AllSteps[j]) == false)
                    return false;
            }
        }
        return true;
    }
}

class A_Star_MDDs_Node
{
    /// <summary>
    /// The last move of all agents that have already moved in this turn.
    /// Used for making sure the next agent move doesn't collide with moves already made.
    /// Used while generating this node, nullified when done.
    /// </summary>
    public HashSet<TimedMove> CurrentMoves { get; set; }
    public MDDNode[] AllSteps { get; set; }
    public A_Star_MDDs_Node Prev { get; set; }
    public int ConflictCount { get; set; }
    public Dictionary<int, int> ConflictCounts { get; set; }
    public Dictionary<int, List<int>> ConflictTimes { get; set; }

    public bool AlreadyExpanded { get; set; }

    /// <summary>
    /// Starts at zero, incremented after a node is expanded once. Set on Expand.
    /// </summary>
    public ushort TargetDeltaConflictCount { get; set; } = 0;
    /// <summary>
    /// Remaining delta conflict count towards targetDeltaConflictCount. Reset on Expand.
    /// </summary>
    public ushort RemainingDeltaConflictCount { get; set; }
    /// <summary>
    /// For each MDD node and each child it has, the effect of that choosing that child on the conflict count.
    /// byte.MaxValue means this is an illegal move. Only computed on demand.
    /// </summary>
    private byte[][] _singleAgentDeltaConflictCounts;
    /// <summary>
    /// Only computed on demand
    /// </summary>
    private ushort _maxDeltaConflictCount;
    /// <summary>
    /// Per each MDD node and delta conflict count, has 1 if that delta F is achievable by choosing child MDD nodes starting from this one on,
    /// -1 if it isn't, and 0 if we don't know yet.
    /// Only computed on demand
    /// </summary>
    private sbyte[][] _conflictCountLookup;

    /// <summary>
    /// From generated nodes. Allows expansion table to be garbage collected before all generated nodes are expanded.
    /// </summary>
    public void ClearExpansionData()
    {
        _singleAgentDeltaConflictCounts = null;
        _conflictCountLookup = null;
        CurrentMoves = null;
    }

    /// <summary>
    /// Counts the number of times this node collides with each agent move in the conflict avoidance table.
    /// </summary>
    public virtual void IncrementConflictCounts(ConflictAvoidanceTable CAT)
    {
        foreach (var mddNode in AllSteps)
        {
            mddNode.move.IncrementConflictCounts(CAT, ConflictCounts, ConflictTimes);
        }
    }

    /// <summary>
    /// Returns whether all possible f values were generated from this node already
    /// </summary>
    public bool hasMoreChildren() => TargetDeltaConflictCount <= _maxDeltaConflictCount;

    public bool IsAlreadyExpanded() => AlreadyExpanded;

    public bool hasChildrenForCurrentDeltaConflictCount(int agentNum = 0) => existsChildForConflictCount(agentNum, RemainingDeltaConflictCount);

    /// <summary>
    /// An MDD child node was chosen between calculating the singleAgentDeltaConflictCounts and this call.
    /// Using the data that describes its delta conflict count potential before the move.
    /// </summary>
    /// <param name="mddIndex">which mdd's node was expanded to create this A*_MDDs node</param>
    /// <param name="childIndex">which of the mdd node's children was just chosen</param>
    public void UpdateRemainingDeltaConflictCount(int mddIndex, int childIndex)
    {
        if (RemainingDeltaConflictCount == ushort.MaxValue)
            Trace.Assert(false,
                            $"Remaining deltaConflictCount is ushort.MaxValue, a reserved value with special meaning. agentIndex={mddIndex}");

        byte lastMoveDeltaConflictCount = _singleAgentDeltaConflictCounts[mddIndex][childIndex];
        if (lastMoveDeltaConflictCount != byte.MaxValue && RemainingDeltaConflictCount >= lastMoveDeltaConflictCount)
            RemainingDeltaConflictCount -= lastMoveDeltaConflictCount;
        else
            RemainingDeltaConflictCount = ushort.MaxValue; // Either because last move was illegal or because the delta F from the last move was more than the entire remaining delta F budget
    }

    /// <summary>
    /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
    /// </summary>
    /// <param name="mddIndex"></param>
    /// <param name="remainingTargetDeltaF"></param>
    /// <returns></returns>
    protected bool existsChildForConflictCount(int mddIndex, ushort remainingTargetDeltaConflictCount)
    {
        // Stopping conditions:
        if (mddIndex == AllSteps.Length)
        {
            if (remainingTargetDeltaConflictCount == 0)
                return true;
            return false;
        }

        if (_conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] != 0) // Answer known (arrays are initialized to zero). TODO: Replace the magic.
        {
            return _conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] == 1; // Return known answer. TODO: Replace the magic
        }

        // Recursive actions:
        for (int i = 0; i < AllSteps[mddIndex].children.Count; i++)
        {
            if (_singleAgentDeltaConflictCounts[mddIndex][i] > remainingTargetDeltaConflictCount) // Small optimization - no need to make the recursive
                                                                                                // call just to request a negative target from it and
                                                                                                // get false (because we assume the heuristic function
                                                                                                // is consistent)
                continue;
            if (existsChildForConflictCount(mddIndex + 1,
                                            (byte)(remainingTargetDeltaConflictCount - _singleAgentDeltaConflictCounts[mddIndex][i])))
            {
                _conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] = 1;
                return true;
            }
        }
        _conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] = -1;
        return false;
    }

    /// <summary>
    /// Calculates for each MDD node and each of its children, the effect of that move on the conflict count.
    /// Also calcs maxDeltaConflictCount.
    /// Note: Currently avoids the CAT's avoidanceGoal. To compute each individual agent's effect on the count of groups we conflict with would require
    /// tracking conflicts as a set of groups we conflict with instead of as a sum of conflicts, and would require 2^(num agents) cells in each singleAgentDeltaConflictCounts[i].
    /// </summary>
    /// <param name="CAT"></param>
    /// <returns></returns>
    public void calcSingleAgentDeltaConflictCounts(ConflictAvoidanceTable CAT)
    {
        // Init
        _singleAgentDeltaConflictCounts = new byte[AllSteps.Length][];
        for (int i = 0; i < AllSteps.Length; i++)
        {
            _singleAgentDeltaConflictCounts[i] = new byte[AllSteps[i].children.Count];
        }

        int conflictCountAfter;

        _maxDeltaConflictCount = 0;

        // Set values
        for (int i = 0; i < AllSteps.Length; i++)
        {
            int singleAgentMaxLegalDeltaConflictCount = -1;

            foreach ((int childIndex, MDDNode child) in AllSteps[i].children.Enumerate())
            {
                if (CAT != null)
                {
                    conflictCountAfter = CAT[child.move].Count;
                }
                else
                    conflictCountAfter = 0;

                _singleAgentDeltaConflictCounts[i][childIndex] = (byte)conflictCountAfter;
                singleAgentMaxLegalDeltaConflictCount = Math.Max(singleAgentMaxLegalDeltaConflictCount, _singleAgentDeltaConflictCounts[i][childIndex]);
            }

            if (singleAgentMaxLegalDeltaConflictCount == -1) // No legal action for this agent, so no legal children exist for this node
            {
                _maxDeltaConflictCount = 0; // Can't make it negative without widening the field.
                break;
            }

            _maxDeltaConflictCount += (byte)singleAgentMaxLegalDeltaConflictCount;
        }

        _conflictCountLookup = new sbyte[AllSteps.Length][];
        for (int i = 0; i < _conflictCountLookup.Length; i++)
        {
            _conflictCountLookup[i] = new sbyte[_maxDeltaConflictCount + 1];  // Towards the last agents most of the row will be wasted (the last one can do delta F of 0 or 1),
                                                                                    // but it's easier than fiddling with array sizes
        }
    }

    public A_Star_MDDs_Node(MDDNode[] allSteps, A_Star_MDDs_Node prevStep)
    { 
        AllSteps = allSteps;
        Prev = prevStep;
        CurrentMoves = null;  // All non-intermediate nodes have currentMoves == null
        ConflictCount = 0;

        // Initialize conflict tracking data structures
        ConflictCounts = [];
        ConflictTimes = [];
    }

    /// <summary>
    /// Copy constructor
    /// </summary>
    public A_Star_MDDs_Node(A_Star_MDDs_Node cpy, bool createIntermediate)
    {
        AllSteps = new MDDNode[cpy.AllSteps.Length];
        for (int i = 0; i < AllSteps.Length; i++)
        {
            AllSteps[i] = cpy.AllSteps[i];
        }
        Prev = cpy.Prev;
        if (cpy.CurrentMoves != null)
        {
            // cpy is an intermediate node
            if (createIntermediate)
                CurrentMoves = [.. cpy.CurrentMoves];
            else
                CurrentMoves = cpy.CurrentMoves;  // We're not going to add anything currentMoves
        }
        else
            // cpy is a concrete node
            CurrentMoves = new HashSet<TimedMove>(capacity: cpy.AllSteps.Length);

        // The conflictTimes and conflictCounts are only copied later if necessary.

        AlreadyExpanded = false; // Creating a new unexpanded node from cpy

        // For intermediate nodes created during expansion (fully expanded nodes have these fields recalculated when they're expanded)
        TargetDeltaConflictCount = cpy.TargetDeltaConflictCount;  // Just to ease debugging
        RemainingDeltaConflictCount = cpy.RemainingDeltaConflictCount;
        _singleAgentDeltaConflictCounts = cpy._singleAgentDeltaConflictCounts; // For the UpdateRemainingDeltaConflictCount call on temporary nodes.
                                                                                // Notice that after an agent is moved its row won't be up-to-date.
        _conflictCountLookup = cpy._conflictCountLookup; // For the hasChildrenForCurrentDeltaConflictCount call on temporary nodes.
                                                        // Notice that after an agent is moved, all rows up to and including the one of the agent that moved
                                                        // won't be up-to-date.
        _maxDeltaConflictCount = cpy._maxDeltaConflictCount; // Not necessarily achievable after some of the agents moved.
                                                            // The above is OK because we won't be using data for agents that already moved.
    }

    /// <summary>
    /// Only compares the steps.
    /// </summary>
    /// <param name="obj"></param>
    /// <returns></returns>
    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        A_Star_MDDs_Node comp = (A_Star_MDDs_Node)obj;
        return AllSteps.SequenceEqual<MDDNode>(comp.AllSteps);
    }

    /// <summary>
    /// Only uses the steps
    /// </summary>
    public override int GetHashCode()
    {
        unchecked
        {
            int code = 0;
            for (int i = 0; i < AllSteps.Length; i++)
            {
                code += AllSteps[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
            }
            return code;
        }
    }

    public int GetDepth() => AllSteps[0].move.Time;

    /// <summary>
    /// Updates the conflictCount member according to given CATs. Table may be null.
    /// </summary>
    public void UpdateConflicts(ConflictAvoidanceTable cat)
    {
        if (Prev == null)
            return;
        if (cat != null)
        {
            for (int i = 0; i < AllSteps.Length; i++)
            {
                if (cat.ContainsKey(AllSteps[i].move))
                    ConflictCount += cat[AllSteps[i].move].Count;
            }
        }
    }

    public class Comparer : IComparer<A_Star_MDDs_Node>
    {
        /// <summary>
        /// Prefers fewer conflicts. If the number of conflicts is the same, prefers more depth.
        /// </summary>
        public int Compare(A_Star_MDDs_Node x, A_Star_MDDs_Node y)
        {
            if (x.ConflictCount + x.TargetDeltaConflictCount < y.ConflictCount + y.TargetDeltaConflictCount)
                return -1;
            if (x.ConflictCount + x.TargetDeltaConflictCount > y.ConflictCount + y.TargetDeltaConflictCount)
                return 1;

            if (x.GetDepth() > y.GetDepth())
                return -1;
            if (x.GetDepth() < y.GetDepth())
                return 1;

            return 0;
        }
    }
}

class A_Star_MDDs_Expander
{
    private A_Star_MDDs_Node a_star_mdd_node;
    /// <summary>
    /// For each MDD, the index of the next child in <children> to choose
    /// </summary>
    int[] chosenChild;

    public A_Star_MDDs_Expander() { }

    public A_Star_MDDs_Expander(A_Star_MDDs_Node a_star_mdd_node)
    {
        a_star_mdd_node = a_star_mdd_node;
        chosenChild = new int[a_star_mdd_node.AllSteps.Length];
        foreach (MDDNode mddNode in a_star_mdd_node.AllSteps)
        {
            if (mddNode.children.Count == 0)
            {
                chosenChild[0] = -1;
                break;
            }
        }
    }

    public void Setup(A_Star_MDDs_Node a_star_mdd_node)
    {
        a_star_mdd_node = a_star_mdd_node;
        chosenChild = new int[a_star_mdd_node.AllSteps.Length];
        foreach (MDDNode mddNode in a_star_mdd_node.AllSteps)
        {
            if (mddNode.children.Count == 0)
            {
                chosenChild[0] = -1;
                break;
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns>The next child, or null if there aren't any more</returns>
    public A_Star_MDDs_Node GetNextChild()
    {
        if (chosenChild[0] == -1)
            return null;
        var mddNodes = new MDDNode[a_star_mdd_node.AllSteps.Length];
        for (int i = 0; i < mddNodes.Length; i++)
        {
            mddNodes[i] = a_star_mdd_node.AllSteps[i].children.ElementAt(chosenChild[i]);
        }
        SetNextChildIndices();
        return new A_Star_MDDs_Node(mddNodes, a_star_mdd_node);
    }

    /// <summary>
    /// Increments the chosenChild indices array by 1
    /// </summary>
    private void SetNextChildIndices()
    {
        SetNextChildIndices(chosenChild.Length - 1);
    }

    private void SetNextChildIndices(int agentNum)
    {
        if (agentNum == -1)
            chosenChild[0] = -1;
        else if (chosenChild[agentNum] < a_star_mdd_node.AllSteps[agentNum].children.Count - 1)
            chosenChild[agentNum]++;
        else
        {
            chosenChild[agentNum] = 0;
            SetNextChildIndices(agentNum - 1);
        }
    }
}
