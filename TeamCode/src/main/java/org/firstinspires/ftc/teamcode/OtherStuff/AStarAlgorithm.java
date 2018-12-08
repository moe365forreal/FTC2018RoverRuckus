package org.firstinspires.ftc.teamcode.OtherStuff;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Methods;

import java.util.ArrayList;

public class AStarAlgorithm {

    public Pair makePair(int fst, int snd) {
        return new Pair(fst, snd);
    }

    public class AltPair {
        public double first;
        public Pair second;

        public AltPair(double fst, Pair snd) {
            this.first = fst;
            this.second = snd;
        }
    }

    public class Cell {
        public int parent_i;
        public int parent_j;
        public double f;
        public double g;
        public double h;

        public Cell(int PI, int PJ, double F, double G, double H) {
            this.parent_i = PI;
            this.parent_j = PJ;
            this.f = F;
            this.g = G;
            this.h = H;
        }
    }

    public static final int ROW = 73;
    public static final int COLUMN = 73;
    public static final String HEURISTIC_TYPE = "EUCLIDIAN"; //Doesn't do anything, just to remind you actively abt heuristic.

    // A Utility Function to check whether given cell (row, col)
    // is a valid cell or not.
    private boolean isValid(int row, int col) {
        return (row >= 0) && (row < ROW) && (col >= 0) && (col < COLUMN);
    }

    // A Utility Function to check whether the given cell is
    // blocked or not
    private boolean isUnBlocked(/*int grid[][COL], int row, int col*/int[][] grid, int row, int col) {
        // Returns true if the cell is not blocked else false
        if (grid[row][col] == 0) {
            return true;
        } else {
            return false;
        }
    }

    // A Utility Function to check whether destination cell has
    // been reached or not
    private boolean isDestination(int row, int col, Pair dest) {
        if (row == dest.first && col == dest.second) {
            return true;
        } else {
            return false;
        }
    }

    final int POINT_DISTANCE_LIMIT = 5;
    ArrayList<PointDistance> pointDistances;
    int[][] rowColOperations = new int[][]{
            {-1, 0}, {1, 0}, {0, 1}, {0, -1}, /*{-1, 1}, /*{-1, -1}, {1, 1}, {1, -1}*/

    };

    private PointDistance getPointDistance(int row, int col, double radius, Telemetry telemetry) {
        pointDistances = new ArrayList<>();
        telemetry.addData("STARTING POINT DISTANCE ACTUAL", "ayy");
        telemetry.addData("row", row);
        telemetry.addData("col", col);
        getPointDistanceActual(row, col, radius, 0, telemetry);
        telemetry.update();

        if (pointDistances.size() > 0) {
            PointDistance minPointDistance = pointDistances.get(0);

            for (int i = 1; i < pointDistances.size(); i++) {
                if (pointDistances.get(i).distance < minPointDistance.distance) {
                    minPointDistance = pointDistances.get(i);
                }
            }

            return minPointDistance;
        } else {
            return null;
        }
    }

    private void getPointDistanceActual(int row, int col, double radius, int runs, Telemetry telemetry) {
        ArrayList<Pair> circleArray = getPointsWithinCircle(col, row, radius);
        boolean robotCanPass = true;
        for (Pair pair : circleArray) {
            if (pair.first < 0 || pair.first >= ROW || pair.second < 0 || pair.second >= COLUMN || AstarAlgorithmMap.mappedFTCField[pair.first][pair.second] == 1) {
                robotCanPass = false;
                break;
            }
        }

        if (isUnBlocked(AstarAlgorithmMap.mappedFTCField, row, col) && robotCanPass) {
            pointDistances.add(new PointDistance(row, col, runs));
            return;
        } else if (runs > POINT_DISTANCE_LIMIT) {
            return;
        }

        for (int[] rowColOperation : rowColOperations) {
            int rowOp = rowColOperation[0] + row;
            int colOp = rowColOperation[1] + col;

            getPointDistanceActual(rowOp, colOp, radius, runs + 1, telemetry);
        }
    }

    // A Utility Function to calculate the 'h' heuristics.
    private double calculateHValue(int row, int col, Pair dest) {
        // Return using the distance formula
        /**
         THERE ARE 3 MAIN HEURISTIC FUNCTIONS: Euclidian, left/right/up/down, and 8 directional - think about 4 directional & euclidian for MOEbot

         this is MANHATTAN: only when moving up, right, left, down
         h = abs (current_cell.x – goal.x) +
         abs (current_cell.y – goal.y)

         this is EUCLIDIAN: only when moving all 8 directions.
         **/
        return Math.sqrt((row - dest.first) * (row - dest.first) + (col - dest.second) * (col - dest.second));
    }

    // A Utility Function to trace the path from the source
    // to destination
    private ArrayList<Pair> tracePath(/*cell cellDetails[][COL], Pair dest*/Cell[][] cellDetails, Pair dest) {
        //System.out.println ("\nThe Path is ");
        int row = dest.first;
        int col = dest.second;

        ArrayList<Pair> path = new ArrayList<>();

        while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
            path.add(new Pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
        }

        path.add(new Pair(row, col));
        // while (path.size() != 0) {
        //     Pair p = path.get(0);
        //     path.remove(0);
        //     //System.out.println("-> (" + p.first + "," + p.second + ") ");
        // }
        return path;
    }

    private ArrayList<Pair> getPointsWithinCircle(double colCenter, double rowCenter, double radius) {
        ArrayList<Pair> pointsWithinCircle = new ArrayList<>();
        for (int x = (int) Math.floor(colCenter - radius); x <= Math.ceil(colCenter + radius); x++) {
            double yspan = radius * Math.sin(Math.acos((colCenter - x) / radius));
            for (int y = (int) Math.floor(rowCenter - yspan); y < Math.ceil(rowCenter + yspan); y++) {
                pointsWithinCircle.add(makePair(y, x));
            }
        }
        return pointsWithinCircle;
    }


    // A Function to find the shortest path between
    // a given source cell to a destination cell according
    // to A* Search Algorithm
    public Pathing aStarSearch(/*int grid[][COL], Pair src, Pair dest*/int[][] grid, Pair src, Pair dest, double robotRadius,
                                                                       Telemetry telemetry) {
        double smallestRadius = robotRadius;
        ArrayList<Pair> returnPathing = new ArrayList<>();
        ArrayList<Pair> pathOutOfBarrier = new ArrayList<>();
        // If the source is out of range
        if (!isValid(src.first, src.second)) {
            //System.out.println ("Source is invalid\n");
            return new Pathing(returnPathing, "NOT VALID SOURCE");
        }

        // If the destination is out of range
        if (!isValid(dest.first, dest.second)) {
            //System.out.println ("Destination is invalid\n");
            return new Pathing(returnPathing, "NOT VALID DESTINATION");
        }

        if (!isUnBlocked(grid, src.first, src.second) || !isUnBlocked(grid, dest.first, dest.second)) {
            return new Pathing(returnPathing, "NO VALID START OR DEST POINTS");
        }

        // Either the source or the destination is blocked
//        boolean robotCanPass = true;
//        for (Pair pair : getPointsWithinCircle(src.second, src.first, robotRadius)) {
//            if (pair.first < 0 || pair.first >= ROW || pair.second < 0 || pair.second >= COLUMN || mappedFTCField[pair.first][pair.second] == 1) {
//                robotCanPass = false;
//                break;
//            }
//        }
//        PointDistance pDist = null;
//        if (/*(!isUnBlocked(grid, src.first, src.second) || !isUnBlocked(grid, dest.first, dest.second)) ||*/// !robotCanPass) {
        //System.out.println ("Source or the destination is blocked\n");
//            pDist = getPointDistance(src.first, src.second, robotRadius, telemetry);
//
//            if (pDist == null) {
//                return new Pathing(returnPathing, "NO VALID POINT DISTANCE");
//            }
//
//            int xDist = pDist.x-src.second;
//            int yDist = pDist.y-src.first;
//
//            if (xDist < 0) {
//                for (int i = -1; i >= xDist; i--) {
//                    pathOutOfBarrier.add(new Pair(src.first, src.second+i));
//                }
//            } else {
//                for (int i = 1; i <= xDist; i++) {
//                    pathOutOfBarrier.add(new Pair(src.first, src.second+i));
//                }
//            }
//
//            if (yDist < 0) {
//                for (int i = -1; i >= yDist; i--) {
//                    pathOutOfBarrier.add(new Pair(src.first+i, pDist.x));
//                }
//            } else {
//                for (int i = 1; i <= yDist; i++) {
//                    pathOutOfBarrier.add(new Pair(src.first+i, pDist.x));
//                }
//            }
//        }

        // If the destination cell is the same as source cell
        if (isDestination(src.first, src.second, dest)) {
            //System.out.println ("We are already at the destination\n");
            return new Pathing(returnPathing, "AT DESTINATION ALREADY");
        }

        // Create a closed list and initialise it to false which means
        // that no cell has been included yet
        // This closed list is implemented as a boolean 2D array
        boolean[][] closedList = new boolean[ROW][COLUMN];
        Cell[][] cellDetails = new Cell[ROW][COLUMN];
        for (int a = 0; a < ROW; a++) {
            for (int b = 0; b < COLUMN; b++) {
                closedList[a][b] = false;

                cellDetails[a][b] = new Cell(-1, -1, Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
            }
        }

        // Initialising the parameters of the starting node
        int i = src.first, j = src.second;
//        if (pDist == null) {
//            i = src.first;
//            j = src.second;
//        } else {
//            i = pDist.y;
//            j = pDist.x;
//        }

        //telemetry.addData("pdist x: " + String.valueOf(pDist.x), "pdist y: " + String.valueOf(pDist.y));
        cellDetails[i][j].f = 0.0;
        cellDetails[i][j].g = 0.0;
        cellDetails[i][j].h = 0.0;
        cellDetails[i][j].parent_i = i;
        cellDetails[i][j].parent_j = j;

        /*
         Create an open list having information as-
         <f, <i, j>>
         where f = g + h,
         and i, j are the row and column index of that cell
         Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
         This open list is implenented as a set of pair of pair.*/
        ArrayList<AltPair> openList = new ArrayList<>();

        // Put the starting cell on the open list and set its
        // 'f' as 0
        openList.add(new AltPair(0.0, new Pair(i, j)));

        // We set this boolean value as false as initially
        // the destination is not reached.
        boolean foundDest = false;

        while (openList.size() != 0) {
            AltPair p = openList.get(0);

            // Remove this vertex from the open list
            openList.remove(0);

            // Add this vertex to the closed list
            i = p.second.first;
            j = p.second.second;
            closedList[i][j] = true;

           /*
            Generating all the 8 successor of this cell

                N.W   N   N.E
                  \   |   /
                   \  |  /
                W----Cell----E
                     / | \
                   /   |  \
                S.W    S   S.E

            Cell-->Popped Cell (i, j)
            N -->  North       (i-1, j)
            S -->  South       (i+1, j)
            E -->  East        (i, j+1)
            W -->  West           (i, j-1)
            N.E--> North-East  (i-1, j+1)
            N.W--> North-West  (i-1, j-1)
            S.E--> South-East  (i+1, j+1)
            S.W--> South-West  (i+1, j-1)*/

            // To store the 'g', 'h' and 'f' of the 8 successors
            double gNew, hNew, fNew;

            //north, south, east, west, north-east, north-west, south-east, south-west
            //purpose of this is to simplify code length, don't have to write 8 if statements for all direcitons.
            int[][] rowColOperations = new int[][]{
                    {-1, 0}, {1, 0}, {0, 1}, {0, -1}, /*{-1, 1}, {-1, -1}, {1, 1}, {1, -1}*/
            };

            boolean pointWasFound = false;
            boolean overrideLoop = false;
            double tempRobotRadius = robotRadius;
            while (!pointWasFound && !overrideLoop) {
                overrideLoop = true;
                for (int[] rowColOperation : rowColOperations) {
                    int checkingI = i + rowColOperation[0];
                    int checkingJ = j + rowColOperation[1];

                    if (isValid(checkingI, checkingJ)) {
                        // If the destination cell is the same as the
                        // current successor
                        if (isDestination(checkingI, checkingJ, dest)) {

                            overrideLoop = false;
                            // Set the Parent of the destination cell
                            cellDetails[checkingI][checkingJ].parent_i = i;
                            cellDetails[checkingI][checkingJ].parent_j = j;
                            //System.out.println ("The destination cell is found\n");
                            returnPathing = tracePath(cellDetails, dest);
                            foundDest = true;

                            return new Pathing(returnPathing, "FOUND DESTINATION");

//                        if (pathOutOfBarrier.size() != 0) {
//                            ArrayList<Pair> newBarrier = new ArrayList<>();
//                            for (Pair pair : pathOutOfBarrier) {
//                                newBarrier.add(pair);
//                            }
//                            pathOutOfBarrier.addAll(returnPathing);
//                            return new Pathing(pathOutOfBarrier, newBarrier, "NEW BARRIER");
//                        } else {
//                            return new Pathing(pathOutOfBarrier, "FOUND DETINATION");
//                        }
                        }
                        // If the successor is already on the closed
                        // list or if it is blocked, then ignore it.
                        // Else do the following
                        else if (!closedList[checkingI][checkingJ] &&
                                isUnBlocked(grid, checkingI, checkingJ)) {

                            overrideLoop = false;

                            boolean found;
                            //so that robotRadius value is kept constant
                            //we don't want to change it.
                            found = true;
                            for (Pair pair : getPointsWithinCircle(checkingJ, checkingI, tempRobotRadius)) {
                                if (pair.first < 0 || pair.first >= ROW || pair.second < 0 || pair.second >= COLUMN || AstarAlgorithmMap.mappedFTCField[pair.first][pair.second] == 1) {
                                    found = false;
                                }
                            }

                            if (found) {
                                pointWasFound = true;
                                gNew = cellDetails[i][j].g + 1.0;
                                hNew = calculateHValue(checkingI, checkingJ, dest);
                                fNew = gNew + hNew;

                                // If it isn’t on the open list, add it to
                                // the open list. Make the current square
                                // the parent of this square. Record the
                                // f, g, and h costs of the square cell
                                //                OR
                                // If it is on the open list already, check
                                // to see if this path to that square is better,
                                // using 'f' cost as the measure.
                                if (cellDetails[checkingI][checkingJ].f == Float.MAX_VALUE ||
                                        cellDetails[checkingI][checkingJ].f > fNew) {
                                    openList.add(new AltPair(fNew, new Pair(checkingI, checkingJ)));

                                    // Update the details of this cell
                                    cellDetails[checkingI][checkingJ].f = fNew;
                                    cellDetails[checkingI][checkingJ].g = gNew;
                                    cellDetails[checkingI][checkingJ].h = hNew;
                                    cellDetails[checkingI][checkingJ].parent_i = i;
                                    cellDetails[checkingI][checkingJ].parent_j = j;
                                }
                            }
                        }
                    }
                }

//                telemetry.addData("radius: ", tempRobotRadius);
//                telemetry.addData("found", pointWasFound);
//                telemetry.update();

                tempRobotRadius -= 0.25;
                if (tempRobotRadius < smallestRadius) {
                    smallestRadius = tempRobotRadius;
                }
            }
        }

        // When the destination cell is not found and the open
        // list is empty, then we conclude that we failed to
        // reach the destiantion cell. This may happen when the
        // there is no way to destination cell (due to blockages)
        if (foundDest == false) {
            //System.out.println("Failed to find the Destination Cell\n");
        }
        telemetry.addData("SMALLEST RADIUS:", smallestRadius);
        return new Pathing(returnPathing, "NO FOUND DESTINATION");
    }

    public Pair convertPairToFtcGrid(Pair pair) {
        return new Pair((int) Math.round(Methods.scaleToRange(pair.first, 0, 72, 0, 59)), (int) Math.round(Methods.scaleToRange(pair.second, 0, 72, 0, 59)));
    }

    // Driver program to test above function
    public static Pathing getAStarRobotPathing(Pair startPair, Pair destinationPair, double robotRadius, Telemetry telemetry) {
        // int grid[][] = new int[][]{
        //     { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
        //     { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }
        // }; - DEBUG

        AStarAlgorithm algo = new AStarAlgorithm();

        return algo.aStarSearch(AstarAlgorithmMap.mappedFTCField, startPair, destinationPair, robotRadius, telemetry);
    }


    public static ArrayList<RobotMovementPair> convertPathingToRobotMovements(Pathing pathing) {
        //first part of pair is direction, second part of pair is inches/analysis units
        ArrayList<RobotMovementPair> movementList = new ArrayList<>();
        Pair lastCoords = pathing.pathing.get(pathing.pathing.size() - 1);

        String knownDirection = "NULL";
        String lastKnownDirection = "NULL";
        int inchesCount = 0;
        for (int i = pathing.pathing.size() - 2; i >= 0; i--) {
            Pair coords = pathing.pathing.get(i);

            int rowDifference = coords.first - lastCoords.first;
            inchesCount += 1;
            if (rowDifference > 0) {
                knownDirection = "UP";
            } else if (rowDifference < 0) {
                knownDirection = "DOWN";
            } else {
                int columnDifference = coords.second - lastCoords.second;

                if (columnDifference < 0) {
                    knownDirection = "LEFT";
                } else if (columnDifference > 0) {
                    knownDirection = "RIGHT";
                }
            }

            if (!knownDirection.equals(lastKnownDirection) && inchesCount != 1) {
                movementList.add(new RobotMovementPair(lastKnownDirection, (inchesCount - 1) * 2));
                inchesCount = 1;
            }

            lastCoords = coords;
            lastKnownDirection = knownDirection;
        }

        movementList.add(new RobotMovementPair(lastKnownDirection, (inchesCount - 1) * 2));

        return movementList;
    }

}
