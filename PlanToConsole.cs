using mapf;
using System;
using System.Collections.Generic;
using System.Linq;


namespace mapfWin;

internal static class PlanToConsole
{
    public static void PrintSolution(bool[][] grid, Plan plan)
    {
        for(int i = 0; i < plan.GetSize(); i++)
        {
            Console.WriteLine($"Time step {i}");
            List<Move> moves = plan.GetLocationsAt(i);
            for (int y = 0; y < grid[0].Length; y++)
            {
                for (int x = 0; x < grid.Length; x++)
                {
                    int agentAtPoint = moves.Select( (m, aNum) => new { m, ANum = aNum + 1 }).Where(pair => pair.m.X == x && pair.m.Y == y).Select(m => m.ANum).FirstOrDefault() -1;
                    if(agentAtPoint >= 0)
                        Console.Write(agentAtPoint);
                    else
                    if (grid[x][y])
                        Console.Write("X");
                    else
                        Console.Write(".");
                }
                Console.WriteLine();
            }
            Console.ReadLine();
        }
    }
}
