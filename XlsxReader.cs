using ClosedXML.Excel;
using mapf;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;

namespace mapfWin;


internal static class XlsxReader
{
    public static ProblemInstance ReadProblemFromXlsx(string filePath)
    {
        string excelPath = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location)!, "Layouts", filePath);
        XLWorkbook xLWorkbook = new(excelPath);
        IXLWorksheet worksheet = xLWorkbook.Worksheets.First();

        // read map size from cells B6 and C6
        int columns = worksheet.Cell("B6").GetValue<int>();
        int rows = worksheet.Cell("C6").GetValue<int>();

        BitMatrix grid = new(columns, rows);

        List<Agent> agents = [];
        List<AgentState> states = [];

        // read matrix: green cells are free (false), red cells are obstacles (true)
        for (int i = 0; i < columns; i++)
        {
            for (int j = 0; j < rows; j++)
            {
                var cell = worksheet.Cell(j + 7, i + 1);
                Color fillColor = cell.Style.Fill.BackgroundColor.Color;
                grid[i, j] = fillColor.ToArgb() == Color.Red.ToArgb();

                string cellText = cell.GetString();

                if (string.IsNullOrWhiteSpace(cellText))
                    continue;

                string[] groups = cellText.Split(['(', ')', ','], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

                if (groups.Length < 3)
                    throw new Exception($"Invalid agent text description: {cellText}");

                Agent agent = new(int.Parse(groups[1]), int.Parse(groups[2]), agents.Count());
                agents.Add(agent);

                states.Add(new AgentState(i, j, agent));
            }
        }


        ProblemInstance problem = new();
        problem.Init([..states], grid);
        problem.ComputeSingleAgentShortestPaths(); // CBS needs it
        return problem;
    }
}
